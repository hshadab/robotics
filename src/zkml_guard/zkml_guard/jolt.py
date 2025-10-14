import hashlib
import json
import os
import shlex
import subprocess
import tempfile
from dataclasses import dataclass
from typing import Any, Dict, Optional


@dataclass
class ProofResult:
    proof_verified: bool
    proof_ms: Optional[float]
    proof_id: Optional[str]
    raw_stdout: Optional[str]
    raw_stderr: Optional[str]
    cmd: str


def sha256_file(path: str) -> str:
    h = hashlib.sha256()
    with open(path, 'rb') as f:
        for chunk in iter(lambda: f.read(1024 * 1024), b''):
            h.update(chunk)
    return h.hexdigest()


def sha256_bytes(b: bytes) -> str:
    return hashlib.sha256(b).hexdigest()


def run_proof(
    model_path: str,
    input_npy: bytes,
    threshold: float,
    label_id: Optional[int],
    prove_cmd_template: str,
    timeout_sec: float = 5.0,
) -> ProofResult:
    """
    Runs the configured JOLT Atlas CLI using a template string.

    Placeholders supported in the template:
      {model}     - absolute model path
      {input}     - absolute path to the preprocessed tensor (.npy)
      {raw_input} - absolute path to the original raw RGB image (.png)
      {threshold} - confidence threshold (float)
      {label}     - integer label id (or empty string if None)
      {output}    - absolute path to a JSON output path (created)

    The CLI should print JSON to stdout with keys like:
      proof_verified (bool), proof_ms (number), proof_id (string)
    Extra fields are ignored.
    """
    os.makedirs(os.path.expanduser('~/.cache/zkml_guard/tmp'), exist_ok=True)
    with tempfile.TemporaryDirectory(dir=os.path.expanduser('~/.cache/zkml_guard/tmp')) as td:
        input_path = os.path.join(td, 'input.npy')
        with open(input_path, 'wb') as f:
            f.write(input_npy)

        # Optional raw PNG if users prefer proving on raw image
        raw_png = os.path.join(td, 'input.png')
        # Caller can choose to ignore {raw_input} in template if not used
        # raw_png is not created here; if needed, caller should pass data or add logic.

        output_path = os.path.join(td, 'proof.json')

        cmd = prove_cmd_template.format(
            model=os.path.abspath(model_path),
            input=input_path,
            raw_input=raw_png,
            threshold=threshold,
            label=(label_id if label_id is not None and label_id >= 0 else ''),
            output=output_path,
        )

        try:
            # Prefer not to invoke a shell; split safely if possible
            args = shlex.split(cmd)
        except Exception:
            args = cmd  # fallback

        try:
            proc = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_sec,
                check=False,
                text=True,
            )
        except subprocess.TimeoutExpired as e:
            return ProofResult(False, None, None, None, f'timeout: {e}', cmd)
        except Exception as e:
            return ProofResult(False, None, None, None, f'error: {e}', cmd)

        proof_verified = False
        proof_ms = None
        proof_id = None

        # Try parse stdout as JSON first
        stdout = proc.stdout.strip() if proc.stdout else ''
        stderr = proc.stderr.strip() if proc.stderr else ''
        if stdout:
            try:
                data = json.loads(stdout)
                if isinstance(data, dict):
                    proof_verified = bool(data.get('proof_verified', False))
                    proof_ms = data.get('proof_ms')
                    proof_id = data.get('proof_id') or data.get('id')
                elif isinstance(data, list) and len(data) > 0 and isinstance(data[0], dict):
                    d0 = data[0]
                    proof_verified = bool(d0.get('proof_verified', False))
                    proof_ms = d0.get('proof_ms')
                    proof_id = d0.get('proof_id') or d0.get('id')
            except json.JSONDecodeError:
                # Best-effort regex-free parse: look for keywords
                if 'verified' in stdout.lower():
                    proof_verified = ('true' in stdout.lower()) or ('ok' in stdout.lower())
        # If output file exists, attempt to parse
        if not proof_verified and os.path.exists(output_path):
            try:
                with open(output_path, 'r') as f:
                    data = json.load(f)
                if isinstance(data, dict):
                    proof_verified = bool(data.get('proof_verified', False))
                    proof_ms = data.get('proof_ms')
                    proof_id = data.get('proof_id') or data.get('id')
            except Exception:
                pass

        return ProofResult(proof_verified, proof_ms, proof_id, stdout, stderr, cmd)


import hashlib
import json
import os
import shlex
import struct
import subprocess
import tempfile
from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple


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


def create_inference_commitment(
    top1_id: int,
    confidence: float,
    model_hash: str,
) -> Tuple[str, bytes]:
    """
    Create a cryptographic commitment to MobileNetV2 inference output.

    This binds the sentinel proof to the actual inference result by creating
    a commitment that includes:
    - top1_id: The argmax class from MobileNetV2
    - confidence: The softmax probability for that class
    - model_hash: The SHA256 of the model (prevents model substitution)

    The sentinel proof will verify that it knows values matching this commitment.

    Args:
        top1_id: Argmax class ID (0-999 for ImageNet)
        confidence: Softmax probability (0.0-1.0)
        model_hash: SHA256 hex string of the ONNX model

    Returns:
        Tuple of (commitment_hex, commitment_bytes)
        - commitment_hex: Hex string for logging/display
        - commitment_bytes: Raw bytes for verification

    Security Properties:
        - Collision-resistant: Infeasible to find different inputs with same commitment
        - Binding: Cannot change inference result after commitment is made
        - Deterministic: Same inputs always produce same commitment
    """
    # Quantize confidence to fixed precision (4 decimal places)
    # This prevents floating point precision issues while maintaining accuracy
    confidence_quantized = int(confidence * 10000)

    # Pack data into deterministic binary format
    # Format: top1_id (4 bytes) + confidence_quantized (4 bytes) + model_hash (32 bytes)
    data = struct.pack('<II', top1_id, confidence_quantized)  # Little-endian unsigned ints
    data += bytes.fromhex(model_hash)

    # Use SHA3-256 for commitment (resistant to length extension attacks)
    commitment_bytes = hashlib.sha3_256(data).digest()
    commitment_hex = commitment_bytes.hex()

    return commitment_hex, commitment_bytes


def verify_inference_commitment(
    commitment_hex: str,
    top1_id: int,
    confidence: float,
    model_hash: str,
) -> bool:
    """
    Verify that inference values match a commitment.

    Args:
        commitment_hex: Expected commitment (hex string)
        top1_id: Claimed argmax class ID
        confidence: Claimed confidence score
        model_hash: SHA256 of the model

    Returns:
        True if commitment matches, False otherwise
    """
    recomputed_hex, _ = create_inference_commitment(top1_id, confidence, model_hash)
    return recomputed_hex == commitment_hex


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

    SECURITY: This function uses shlex.split() to safely parse command templates.
    NEVER pass shell=True to subprocess.run to prevent command injection.
    """
    # Use system temp directory for security (proper permissions, cleanup)
    with tempfile.TemporaryDirectory() as td:
        input_path = os.path.join(td, 'input.npy')
        with open(input_path, 'wb') as f:
            f.write(input_npy)

        # Optional raw PNG if users prefer proving on raw image
        raw_png = os.path.join(td, 'input.png')
        # Caller can choose to ignore {raw_input} in template if not used
        # raw_png is not created here; if needed, caller should pass data or add logic.

        output_path = os.path.join(td, 'proof.json')

        # Format template with safe values
        cmd_str = prove_cmd_template.format(
            model=os.path.abspath(model_path),
            input=input_path,
            raw_input=raw_png,
            threshold=threshold,
            label=(label_id if label_id is not None and label_id >= 0 else ''),
            output=output_path,
        )

        # Parse command string into safe argument list
        # SECURITY: Always use list form, NEVER shell=True
        try:
            args = shlex.split(cmd_str)
        except Exception as e:
            # If shlex parsing fails, the template is malformed - FAIL IMMEDIATELY
            error_msg = f'Invalid command template - parsing failed: {e}'
            return ProofResult(False, None, None, None, error_msg, cmd_str)

        # Validate that we have at least a command to run
        if not args:
            error_msg = 'Command template produced empty command'
            return ProofResult(False, None, None, None, error_msg, cmd_str)

        try:
            # Execute with list of args (safe - no shell interpretation)
            # shell=False is the default, but we make it explicit for security
            proc = subprocess.run(
                args,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                timeout=timeout_sec,
                check=False,
                text=True,
                shell=False,  # CRITICAL: Never use shell=True (prevents injection)
            )
        except subprocess.TimeoutExpired as e:
            return ProofResult(False, None, None, None, f'timeout: {e}', cmd_str)
        except FileNotFoundError as e:
            error_msg = f'Proof binary not found: {args[0]} - {e}'
            return ProofResult(False, None, None, None, error_msg, cmd_str)
        except Exception as e:
            return ProofResult(False, None, None, None, f'error: {e}', cmd_str)

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


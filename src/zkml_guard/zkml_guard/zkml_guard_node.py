import io
import json
import os
import time
import urllib.request
from dataclasses import asdict, dataclass
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from std_msgs.msg import Bool, String
from sensor_msgs.msg import Image

import onnxruntime as ort
from PIL import Image as PILImage

from .jolt import run_proof, sha256_file, sha256_bytes
import requests


IMAGENET_LABELS_URL = (
    'https://raw.githubusercontent.com/pytorch/hub/master/imagenet_classes.txt'
)
MOBILENETV2_ONNX_URL = (
    'https://github.com/onnx/models/raw/main/vision/classification/mobilenet/model/mobilenetv2-7.onnx'
)


def ensure_cached(path: str, url: str) -> str:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path):
        with urllib.request.urlopen(url) as resp:
            data = resp.read()
        with open(path, 'wb') as f:
            f.write(data)
    return path


def softmax(x: np.ndarray) -> np.ndarray:
    x = x - np.max(x)
    e = np.exp(x)
    return e / np.sum(e)


def decode_ros_image(msg: Image) -> np.ndarray:
    """Return HxWx3 uint8 RGB numpy array from ROS Image without cv_bridge."""
    h = msg.height
    w = msg.width
    step = msg.step
    data = np.frombuffer(msg.data, dtype=np.uint8)
    if msg.encoding in ('rgb8', 'rgb_8'):
        arr = data.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
        return arr
    elif msg.encoding in ('bgr8', 'bgr_8'):
        arr = data.reshape((h, step))[:, : w * 3].reshape((h, w, 3))
        return arr[:, :, ::-1]
    elif msg.encoding in ('mono8', '8UC1'):
        arr = data.reshape((h, step))[:, : w].reshape((h, w))
        rgb = np.stack([arr, arr, arr], axis=-1)
        return rgb
    else:
        # Fallback: try assume tightly packed RGB
        arr = data.reshape((h, w, -1))
        if arr.shape[2] >= 3:
            return arr[:, :, :3]
        # Last resort: raise
        raise ValueError(f'Unsupported image encoding: {msg.encoding}')


def preprocess_rgb_for_mobilenet(rgb: np.ndarray) -> np.ndarray:
    # Resize to 224x224, normalize ImageNet mean/std, NCHW float32
    img = PILImage.fromarray(rgb)
    img = img.resize((224, 224), PILImage.BILINEAR)
    arr = np.asarray(img).astype(np.float32) / 255.0
    mean = np.array([0.485, 0.456, 0.406], dtype=np.float32)
    std = np.array([0.229, 0.224, 0.225], dtype=np.float32)
    arr = (arr - mean) / std
    arr = np.transpose(arr, (2, 0, 1))  # HWC->CHW
    arr = np.expand_dims(arr, axis=0)   # NCHW
    return arr


@dataclass
class InferenceResult:
    top1_id: int
    top1_label: Optional[str]
    top1_score: float
    logits: List[float]


class ZkmlGuardNode(Node):
    def __init__(self) -> None:
        super().__init__('zkml_guard')
        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )

        # Parameters
        self.declare_parameter('camera_topic', '/image')
        self.declare_parameter('stop_topic', '/zkml/stop')
        self.declare_parameter('event_topic', '/zkml/event')
        self.declare_parameter('gating_mode', 'argmax')  # 'argmax' or 'threshold'
        self.declare_parameter('threshold', 0.6)
        self.declare_parameter('target_label', '')
        self.declare_parameter('target_label_id', -1)
        self.declare_parameter('require_proof', True)
        self.declare_parameter('inference_period_ms', 500)
        self.declare_parameter('proof_timeout_ms', 5000)
        self.declare_parameter('prove_on', 'rising_edge')  # 'rising_edge' or 'every_pass'
        self.declare_parameter('unlock_hold_ms', 3000)
        self.declare_parameter('reprove_on_label_change', True)
        self.declare_parameter('verifier_mode', 'cli')  # 'cli' or 'http'
        self.declare_parameter('verifier_url', '')      # e.g., http://localhost:9100/verify
        self.declare_parameter('prove_cmd_template',
            # Example — adjust to your jolt-atlas CLI (expects JSON on stdout)
            'jolt-atlas onnx-prove --model {model} --input_npy {input} '
            '--assert-argmax {label} --verify --json')
        cache_root = os.path.expanduser('~/.cache/zkml_guard')
        self.declare_parameter('model_path', os.path.join(cache_root, 'models/mobilenetv2-7.onnx'))
        self.declare_parameter('labels_path', os.path.join(cache_root, 'imagenet_classes.txt'))

        self.camera_topic = self.get_parameter('camera_topic').get_parameter_value().string_value
        self.stop_topic = self.get_parameter('stop_topic').get_parameter_value().string_value
        self.event_topic = self.get_parameter('event_topic').get_parameter_value().string_value
        self.gating_mode = self.get_parameter('gating_mode').get_parameter_value().string_value
        self.threshold = self.get_parameter('threshold').get_parameter_value().double_value
        self.target_label = self.get_parameter('target_label').get_parameter_value().string_value.strip()
        self.target_label_id = self.get_parameter('target_label_id').get_parameter_value().integer_value
        self.require_proof = self.get_parameter('require_proof').get_parameter_value().bool_value
        self.infer_period = self.get_parameter('inference_period_ms').get_parameter_value().integer_value / 1000.0
        self.proof_timeout = self.get_parameter('proof_timeout_ms').get_parameter_value().integer_value / 1000.0
        self.prove_on = self.get_parameter('prove_on').get_parameter_value().string_value
        self.unlock_hold = self.get_parameter('unlock_hold_ms').get_parameter_value().integer_value / 1000.0
        self.reprove_on_label_change = self.get_parameter('reprove_on_label_change').get_parameter_value().bool_value
        self.verifier_mode = self.get_parameter('verifier_mode').get_parameter_value().string_value
        self.verifier_url = self.get_parameter('verifier_url').get_parameter_value().string_value
        self.prove_cmd_template = self.get_parameter('prove_cmd_template').get_parameter_value().string_value
        self.model_path = self.get_parameter('model_path').get_parameter_value().string_value
        self.labels_path = self.get_parameter('labels_path').get_parameter_value().string_value

        # Auto-download model + labels into cache
        try:
            ensure_cached(self.model_path, MOBILENETV2_ONNX_URL)
        except Exception as e:
            self.get_logger().warn(f'Failed to fetch ONNX model: {e}')
            # Fallback to consolidated local model if present
            workspace_root = os.path.expanduser('~/robotics')
            alt_model = os.path.join(workspace_root, 'tools/onnx-verifier/models/mobilenetv2-12.onnx')
            if os.path.exists(alt_model):
                self.model_path = alt_model
                self.get_logger().info(f'Using local model: {alt_model}')
        try:
            ensure_cached(self.labels_path, IMAGENET_LABELS_URL)
        except Exception as e:
            self.get_logger().warn(f'Failed to fetch labels: {e}')

        # Load labels
        self.labels = []
        if os.path.exists(self.labels_path):
            try:
                with open(self.labels_path, 'r') as f:
                    self.labels = [line.strip() for line in f.readlines() if line.strip()]
            except Exception as e:
                self.get_logger().warn(f'Could not read labels: {e}')

        # Resolve target label id if a label string is given
        if self.target_label and self.labels:
            try:
                # naive fallback: find first label containing substring
                for i, lab in enumerate(self.labels):
                    if self.target_label.lower() in lab.lower():
                        self.target_label_id = i
                        break
            except Exception:
                pass

        # Load ONNX session
        try:
            self.sess = ort.InferenceSession(self.model_path, providers=['CPUExecutionProvider'])
            self.input_name = self.sess.get_inputs()[0].name
            self.output_name = self.sess.get_outputs()[0].name
        except Exception as e:
            self.get_logger().error(f'Failed to initialize ONNX runtime: {e}')
            raise

        self.model_sha256 = sha256_file(self.model_path) if os.path.exists(self.model_path) else ''
        # Store the ORIGINAL model hash for tampering detection
        self.original_model_sha256 = self.model_sha256
        self.get_logger().info(f'Model hash registered: {self.original_model_sha256[:16]}...')

        # ROS I/O
        self.image_sub = self.create_subscription(Image, self.camera_topic, self.image_cb, qos)
        self.stop_pub = self.create_publisher(Bool, self.stop_topic, 10)
        self.event_pub = self.create_publisher(String, self.event_topic, 10)

        self.last_infer_ts = 0.0
        self.latest_rgb: Optional[np.ndarray] = None
        self.last_stop: Optional[bool] = None
        self.last_predicate_met: bool = False
        self.last_label_id: Optional[int] = None
        self.verified_until: float = 0.0
        self.verified_label_id: Optional[int] = None
        self.proof_in_progress: bool = False  # Track if proof is currently generating
        self.proving_label_id: Optional[int] = None  # Track which label we're proving

        self.create_timer(self.infer_period, self.tick)
        self.get_logger().info('zkml_guard ready: gating /cmd_vel via /zkml/stop')

    def image_cb(self, msg: Image) -> None:
        try:
            self.latest_rgb = decode_ros_image(msg)
        except Exception as e:
            self.get_logger().warn(f'Image decode failed: {e}')

    def run_inference(self, rgb: np.ndarray) -> InferenceResult:
        inp = preprocess_rgb_for_mobilenet(rgb)
        out = self.sess.run([self.output_name], {self.input_name: inp})[0].squeeze()
        probs = softmax(out)
        top1 = int(np.argmax(probs))
        score = float(probs[top1])
        label = None
        if self.labels and top1 < len(self.labels):
            label = self.labels[top1]
        return InferenceResult(top1, label, score, out.astype(float).tolist())

    def publish_stop(self, stop: bool) -> None:
        if self.last_stop is None or self.last_stop != stop:
            self.last_stop = stop
            self.stop_pub.publish(Bool(data=stop))

    def publish_event(self, payload: dict) -> None:
        self.event_pub.publish(String(data=json.dumps(payload)))

    def tick(self) -> None:
        now = time.time()
        if self.latest_rgb is None:
            # No image yet; keep stop engaged
            self.publish_stop(True)
            return
        if now - self.last_infer_ts < self.infer_period:
            return
        self.last_infer_ts = now

        try:
            res = self.run_inference(self.latest_rgb)
        except Exception as e:
            self.get_logger().warn(f'Inference failed: {e}')
            self.publish_stop(True)
            return

        predicate_met = False
        dynamic_label_used = False
        label_match = (self.target_label_id >= 0 and res.top1_id == self.target_label_id)
        if self.gating_mode.lower() == 'argmax':
            if self.target_label_id >= 0:
                # Static argmax target
                predicate_met = label_match and (res.top1_score >= self.threshold)
            else:
                # Dynamic argmax: accept whichever top1 wins, guarded by threshold
                predicate_met = (res.top1_score >= self.threshold)
                dynamic_label_used = True
        else:
            # Threshold-only gating (no label constraint)
            predicate_met = (res.top1_score >= self.threshold)

        # Prepare proof input (preprocessed tensor)
        try:
            inp = preprocess_rgb_for_mobilenet(self.latest_rgb)
            npy_bytes = io.BytesIO()
            np.save(npy_bytes, inp)
            input_sha = sha256_bytes(npy_bytes.getvalue())
        except Exception as e:
            self.get_logger().warn(f'Failed to serialize input: {e}')
            input_sha = ''
            npy_bytes = None

        proof_verified = False
        proof_ms = None
        proof_id = None
        prove_cmd = None

        # Decide if we should request a new proof this tick
        should_prove = False
        label_for_cmd = None
        if self.gating_mode.lower() == 'argmax':
            label_for_cmd = self.target_label_id if (self.target_label_id >= 0) else res.top1_id
        # Rising edge logic: prove when predicate goes false->true or when label changed (if enabled) or when previous proof expired
        rising_edge = predicate_met and not self.last_predicate_met
        label_changed = (self.last_label_id is not None and res.top1_id != self.last_label_id)
        expired = (time.time() > self.verified_until)
        # IMPORTANT: Don't start a new proof if one is already in progress
        if self.require_proof and predicate_met and npy_bytes is not None and not self.proof_in_progress:
            if self.prove_on.lower() == 'every_pass':
                should_prove = True
            else:
                if rising_edge or expired or (self.reprove_on_label_change and label_changed):
                    should_prove = True

        if should_prove:
            # Mark proof as in progress to prevent concurrent proof attempts
            self.proof_in_progress = True
            self.proving_label_id = label_for_cmd  # Lock in the label we're proving

            # TAMPERING DETECTION: Check if model has been modified
            current_hash = sha256_file(self.model_path) if os.path.exists(self.model_path) else ''
            if current_hash != self.original_model_sha256:
                self.get_logger().error(f'MODEL TAMPERING DETECTED! Expected: {self.original_model_sha256[:16]}... | Actual: {current_hash[:16]}...')
                self.get_logger().error('Proof generation BLOCKED - model hash mismatch')
                proof_verified = False
                proof_ms = None
                proof_id = 'BLOCKED_TAMPERED_MODEL'
                self.proof_in_progress = False
                self.proving_label_id = None
            else:
                self.get_logger().info(f'Starting proof generation for label "{res.top1_label}" (ID: {label_for_cmd}, confidence: {res.top1_score:.1%})')
                # Invoke verifier (HTTP or CLI)
                try:
                    if self.verifier_mode.lower() == 'http' and self.verifier_url:
                        pr = self._run_http_proof(self.model_path, inp, self.proof_timeout)
                    else:
                        pr = run_proof(
                            model_path=self.model_path,
                            input_npy=npy_bytes.getvalue(),
                            threshold=self.threshold,
                            label_id=label_for_cmd,
                            prove_cmd_template=self.prove_cmd_template,
                            timeout_sec=self.proof_timeout,
                        )
                    proof_verified = pr.proof_verified
                    proof_ms = pr.proof_ms
                    proof_id = pr.proof_id
                    prove_cmd = pr.cmd
                    if proof_verified and label_for_cmd is not None:
                        self.verified_label_id = int(label_for_cmd)
                        self.verified_until = time.time() + max(0.0, self.unlock_hold)
                        self.get_logger().info(f'Proof verified successfully in {proof_ms}ms')
                    else:
                        self.get_logger().warn(f'Proof verification failed')
                except Exception as e:
                    self.get_logger().warn(f'Proof invocation failed: {e}')
                    proof_verified = False
                finally:
                    # Always clear the in-progress flag when done (success or failure)
                    self.proof_in_progress = False
                    self.proving_label_id = None

        # Gate motion
        # Allow motion when:
        # - predicate is met, and
        #   - no proof required, or
        #   - we have a recent verified proof for the current label and it hasn't expired, or
        #   - a proof is currently being generated (ignore current camera state until proof completes)
        allow_motion = False

        # IMPORTANT: If proof is in progress, DON'T re-evaluate predicate - just wait for proof to complete
        if self.proof_in_progress:
            # Proof is actively generating - ignore current inference results
            # Keep allowing motion (or keep blocking) based on previous state
            # This prevents cancelling proofs due to confidence fluctuations
            allow_motion = False  # Keep robot stopped while proof generates
            self.get_logger().debug(f'Proof in progress for label ID {self.proving_label_id}, ignoring current detection')
        elif predicate_met:
            if not self.require_proof:
                allow_motion = True
            else:
                # If this tick delivered a fresh proof, allow
                if proof_verified:
                    allow_motion = True
                else:
                    # Otherwise rely on cached verification
                    if time.time() <= self.verified_until:
                        if self.gating_mode.lower() == 'argmax':
                            current_label = label_for_cmd if label_for_cmd is not None else res.top1_id
                            if self.verified_label_id is not None and current_label == self.verified_label_id:
                                allow_motion = True
                        else:
                            # Threshold mode: no label constraint, accept within hold window
                            allow_motion = True
        self.publish_stop(not allow_motion)

        # Publish event metadata
        event = {
            'ts': now,
            'model_path': self.model_path,
            'model_sha256': self.model_sha256,
            'input_sha256': input_sha,
            'top1_id': res.top1_id,
            'top1_label': res.top1_label,
            'top1_score': res.top1_score,
            'gating_mode': self.gating_mode,
            'threshold': self.threshold,
            'label_required': (self.target_label or self.target_label_id >= 0),
            'dynamic_label_used': dynamic_label_used,
            'predicate_met': predicate_met,
            'prove_on': self.prove_on,
            'unlock_hold_ms': int(self.unlock_hold * 1000),
            'verified_label_id': self.verified_label_id,
            'verified_until': self.verified_until,
            'require_proof': self.require_proof,
            'proof_verified': proof_verified,
            'proof_ms': proof_ms,
            'proof_id': proof_id,
            'prove_cmd': prove_cmd,
        }
        self.publish_event(event)

        # Track state for next tick
        self.last_predicate_met = predicate_met
        self.last_label_id = res.top1_id

    def _run_http_proof(self, model_path: str, preprocessed: np.ndarray, timeout_sec: float):
        from .jolt import ProofResult
        url = self.verifier_url
        # IMPORTANT: Verifier expects testInputs to be array of FLATTENED arrays
        # preprocessed shape is [1, 3, 224, 224] = 150528 elements
        # Flatten to 1D array, then wrap in outer array for testInputs format: [[150528 elements]]
        # The verifier will reconstruct the shape based on the model's expected input shape
        flat_input = preprocessed.flatten().astype(float).tolist()  # Flatten to 1D
        test_inputs = [flat_input]  # Wrap in array: [[150528 floats]]

        files = {
            'model': (os.path.basename(model_path), open(model_path, 'rb'), 'application/octet-stream')
        }
        data = {
            'testInputs': json.dumps(test_inputs)  # Array of test cases (each case is a flat array)
        }
        try:
            resp = requests.post(url, files=files, data=data, timeout=timeout_sec)
        except Exception as e:
            try:
                files['model'][1].close()
            except Exception:
                pass
            return ProofResult(False, None, None, None, f'http error: {e}', f'POST {url}')
        finally:
            try:
                files['model'][1].close()
            except Exception:
                pass
        if resp.status_code != 200:
            error_msg = f'HTTP {resp.status_code}: {resp.text[:200]}'
            self.get_logger().error(f'Verifier error: {error_msg}')
            return ProofResult(False, None, None, resp.text, f'status={resp.status_code}', f'POST {url}')
        try:
            js = resp.json()
        except Exception as e:
            self.get_logger().error(f'Failed to parse verifier response: {e}')
            return ProofResult(False, None, None, resp.text, f'json error: {e}', f'POST {url}')

        # Log the response for debugging
        success = js.get('success', False)
        if not success:
            error = js.get('error', 'Unknown error')
            self.get_logger().error(f'Verifier returned error: {error}')
        else:
            # Log the full response structure for debugging
            self.get_logger().info(f'Verifier response keys: {list(js.keys())}')

        proof_verified = False
        proof_ms = None
        proof_id = None
        if isinstance(js, dict):
            pd = js.get('proofData') or js.get('proof') or {}
            cp = pd.get('cryptographicProof') if isinstance(pd, dict) else {}
            if isinstance(cp, dict):
                proof_verified = bool(cp.get('verified', False))
                perf = pd.get('performance') if isinstance(pd, dict) else None
                if isinstance(perf, dict):
                    proof_ms = perf.get('proofGenerationMs')
                # DEBUG: Log what we found
                self.get_logger().info(f'Found cryptographicProof.verified={cp.get("verified")}, proof_ms={proof_ms}')
            else:
                self.get_logger().warn(f'cryptographicProof not found or not a dict. proofData type: {type(pd)}, keys: {pd.keys() if isinstance(pd, dict) else "N/A"}')
            proof_id = js.get('proofHash') or js.get('verificationId')
        return ProofResult(proof_verified, proof_ms, proof_id, resp.text, None, f'POST {url}')


def main(args=None):
    rclpy.init(args=args)
    node = ZkmlGuardNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

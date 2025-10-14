use std::{fs::File, path::PathBuf, time::Instant};

use ark_bn254::Fr;
use clap::Parser;
use ndarray::ArrayD;
use ndarray_npy::read_npy;
use onnx_tracer::{self, tensor::Tensor};
use serde::Serialize;
use sha3::{Digest, Sha3_256};
use zkml_jolt_core::{
    jolt::{
        execution_trace::jolt_execution_trace, JoltProverPreprocessing, JoltSNARK,
    },
};
use jolt_core::{poly::commitment::dory::DoryCommitmentScheme, utils::transcript::KeccakTranscript};

type PCS = DoryCommitmentScheme<KeccakTranscript>;

#[derive(Parser, Debug)]
#[command(name = "atlas_argmax_prover")]
#[command(about = "Argmax proof/verify for ONNX via JOLT Atlas", long_about = None)]
struct Args {
    /// Path to ONNX model
    #[arg(long)]
    model: PathBuf,

    /// Path to input tensor .npy (f32, NCHW)
    #[arg(long)]
    input_npy: PathBuf,

    /// Class id to assert equals argmax. If omitted, just prove execution; returns predicted argmax.
    #[arg(long)]
    assert_argmax: Option<usize>,

    /// Input scale (2^scale) for fixed-point quantization
    #[arg(long, default_value_t = 7)]
    input_scale: i32,

    /// Verify the proof after proving
    #[arg(long, default_value_t = true)]
    verify: bool,

    /// Emit JSON to stdout
    #[arg(long, default_value_t = true)]
    json: bool,
}

#[derive(Serialize)]
struct OutputJson<'a> {
    proof_verified: bool,
    proof_ms: Option<u128>,
    proof_id: Option<String>,
    argmax: Option<usize>,
    assert_label: Option<usize>,
    input_shape: Vec<usize>,
    input_scale: i32,
    model: &'a str,
}

fn quantize_to_i32(arr: &ArrayD<f32>, scale_log2: i32) -> Vec<i32> {
    let scale = 2f32.powi(scale_log2);
    arr.iter()
        .map(|&x| {
            let val = (x * scale).round();
            if val > i32::MAX as f32 {
                i32::MAX
            } else if val < i32::MIN as f32 {
                i32::MIN
            } else {
                val as i32
            }
        })
        .collect()
}

fn argmax_i32(v: &[i32]) -> usize {
    let mut best_i = 0usize;
    let mut best_v = i32::MIN;
    for (i, &val) in v.iter().enumerate() {
        if val > best_v {
            best_v = val;
            best_i = i;
        }
    }
    best_i
}

fn main() -> anyhow::Result<()> {
    let args = Args::parse();

    // Load npy as ArrayD<f32>
    let input_arr: ArrayD<f32> = read_npy(&args.input_npy)?;
    let shape: Vec<usize> = input_arr.shape().to_vec();
    let input_i32: Vec<i32> = quantize_to_i32(&input_arr, args.input_scale);
    let input_tensor = Tensor::<i32>::new(Some(&input_i32), &shape)
        .map_err(|e| anyhow::anyhow!("tensor error: {e}"))?;

    // Load model and build program
    let model_path = args.model.canonicalize()?;
    let mut model = onnx_tracer::model(&model_path.clone());

    // Forward to get output and execution trace
    let (raw_trace, program_output) = onnx_tracer::execution_trace(model.clone(), &input_tensor);
    let output_vec = program_output.output.inner.clone();
    let y_argmax = argmax_i32(&output_vec);

    // Optional local assertion
    if let Some(lbl) = args.assert_argmax {
        if y_argmax != lbl {
            if args.json {
                let out = OutputJson {
                    proof_verified: false,
                    proof_ms: None,
                    proof_id: None,
                    argmax: Some(y_argmax),
                    assert_label: Some(lbl),
                    input_shape: shape.clone(),
                    input_scale: args.input_scale,
                    model: model_path.to_string_lossy().as_ref(),
                };
                println!("{}", serde_json::to_string(&out)?);
                return Ok(());
            } else {
                eprintln!(
                    "Local argmax {} != asserted {} — refusing to prove",
                    y_argmax, lbl
                );
                std::process::exit(2);
            }
        }
    }

    // Prove and (optionally) verify
    let program_bytecode = onnx_tracer::decode_model(model);
    let pp: JoltProverPreprocessing<Fr, PCS, KeccakTranscript> =
        JoltSNARK::prover_preprocess(program_bytecode);
    let exec_trace = jolt_execution_trace(raw_trace);

    let t0 = Instant::now();
    let snark: JoltSNARK<Fr, PCS, KeccakTranscript> =
        JoltSNARK::prove(pp.clone(), exec_trace, &program_output);
    let proof_ms = t0.elapsed().as_millis();

    let mut proof_verified = false;
    if args.verify {
        let verify_res = snark.verify((&pp).into(), program_output);
        proof_verified = verify_res.is_ok();
    }

    // Hash proof bytes as an ID
    let mut hasher = Sha3_256::new();
    let mut proof_bytes = Vec::new();
    snark
        .serialize_compressed(&mut proof_bytes)
        .map_err(|e| anyhow::anyhow!("serialize proof: {e}"))?;
    hasher.update(&proof_bytes);
    let proof_id = format!("sha3-256:{:x}", hasher.finalize());

    if args.json {
        let out = OutputJson {
            proof_verified,
            proof_ms: Some(proof_ms),
            proof_id: Some(proof_id),
            argmax: Some(y_argmax),
            assert_label: args.assert_argmax,
            input_shape: shape,
            input_scale: args.input_scale,
            model: model_path.to_string_lossy().as_ref(),
        };
        println!("{}", serde_json::to_string(&out)?);
    } else {
        println!(
            "proof_verified={} proof_ms={} argmax={} proof_id={}",
            proof_verified, proof_ms, y_argmax, proof_id
        );
    }

    Ok(())
}


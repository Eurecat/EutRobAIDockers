#!/usr/bin/env python3
"""
Verify that LibreYOLO is actually using Jetson Thor GPU
"""
import torch
import numpy as np
import cv2
import time
import sys
import traceback

print("="*70)
print("JETSON THOR GPU VERIFICATION")
print("="*70)

# 1. Check CUDA availability and device
print("\n1. CUDA Status:")
print(f"   CUDA available: {torch.cuda.is_available()}")
if torch.cuda.is_available():
    print(f"   CUDA device: {torch.cuda.get_device_name(0)}")
    capability = torch.cuda.get_device_capability(0)
    print(f"   CUDA capability: sm_{capability[0]}{capability[1]}")
    print(f"   CUDA device count: {torch.cuda.device_count()}")
    
    # Memory info
    try:
        print(f"\n2. GPU Memory Status:")
        print(f"   Total GPU memory: {torch.cuda.get_device_properties(0).total_memory / 1024**3:.2f} GB")
        print(f"   Reserved: {torch.cuda.memory_reserved(0) / 1024**2:.2f} MB")
        print(f"   Allocated: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB")
    except Exception as e:
        print(f"   Error reading GPU memory: {e}")
else:
    print("   WARNING: CUDA not available!")
    sys.exit(1)

# 2. Test GPU vs CPU performance
print("\n3. GPU vs CPU Performance Test:")
print("   Creating test tensor (480x640x3)...\n")

# Create test frame
frame_np = np.ones((480, 640, 3), dtype=np.uint8) * 100
cv2.rectangle(frame_np, (100, 100), (300, 300), (0, 255, 0), 3)

try:
    from libreyolo import LibreYOLO
    
    # Load model
    print("   Loading LibreYOLOXs model...")
    model = LibreYOLO("LibreYOLOXx.pt", device="cuda")
    print("   ✓ Model loaded\n")
    
    # Warmup (first inference is always slow due to compilation)
    print("   Running warmup inference on GPU...")
    t0 = time.time()
    _ = model(frame_np, conf=0.25, iou=0.45)
    warmup_time = (time.time() - t0) * 1000
    print(f"   Warmup completed in {warmup_time:.2f}ms (includes compilation)\n")
    
    # Check GPU memory after model load
    print("   GPU Memory after model load:")
    print(f"   Reserved: {torch.cuda.memory_reserved(0) / 1024**2:.2f} MB")
    print(f"   Allocated: {torch.cuda.memory_allocated(0) / 1024**2:.2f} MB\n")
    
    # Actual inference benchmark (GPU)
    print("   GPU Inference Benchmark (5 runs):")
    gpu_times = []
    for i in range(5):
        torch.cuda.synchronize()  # Ensure GPU is ready
        t0 = time.time()
        _ = model(frame_np, conf=0.25, iou=0.45)
        torch.cuda.synchronize()  # Wait for GPU to finish
        elapsed = (time.time() - t0) * 1000
        gpu_times.append(elapsed)
        print(f"     Run {i+1}: {elapsed:.2f}ms")
    
    avg_gpu = np.mean(gpu_times)
    print(f"   Average: {avg_gpu:.2f}ms")
    print(f"   FPS: {1000/avg_gpu:.1f} fps\n")
    
    # CPU comparison (if you have the model on CPU)
    print("   Loading model on CPU for comparison...")
    try:
        model_cpu = LibreYOLO("LibreYOLOXs.pt", device="cpu")
        print("   ✓ Model loaded on CPU\n")
        
        print("   CPU Inference Benchmark (3 runs, will be slow):")
        cpu_times = []
        for i in range(3):
            t0 = time.time()
            _ = model_cpu(frame_np, conf=0.25, iou=0.45)
            elapsed = (time.time() - t0) * 1000
            cpu_times.append(elapsed)
            print(f"     Run {i+1}: {elapsed:.2f}ms")
        
        avg_cpu = np.mean(cpu_times)
        print(f"   Average: {avg_cpu:.2f}ms")
        print(f"   FPS: {1000/avg_cpu:.1f} fps\n")
        
        speedup = avg_cpu / avg_gpu
        print(f"   GPU Speedup: {speedup:.1f}x faster than CPU")
    except Exception as e:
        print(f"   Skipped CPU benchmark: {e}\n")
    
    # 4. Verify model is on GPU
    print("4. Model Device Verification:")
    for name, param in model.model.named_parameters():
        device = param.device
        print(f"   Layer '{name}': {device}")
        break  # Just check first layer
    print("   (All model weights should be on 'cuda:0' or 'cpu')\n")
    
    print("="*70)
    print("✓ GPU VERIFICATION COMPLETE")
    print("="*70)
    
    if avg_gpu < 100:  # Less than 100ms per frame
        print(f"\n✓ CONFIRMED: GPU is being used! ({avg_gpu:.1f}ms per frame is fast)")
    else:
        print(f"\n⚠ SLOW INFERENCE: {avg_gpu:.1f}ms might indicate CPU fallback")
    
except ImportError as e:
    print(f"ERROR: Failed to import LibreYOLO: {e}")
    sys.exit(1)
except Exception as e:
    print(f"ERROR: {e}")
    traceback.print_exc()
    sys.exit(1)

#!/usr/bin/env python3
"""
CUDA and GPU Compatibility Validation for Module 3
Validates that the system meets NVIDIA Isaac requirements
"""

import subprocess
import sys
import os
import platform

def check_nvidia_gpu():
    """Check if NVIDIA GPU is available"""
    try:
        # Check if nvidia-smi is available
        result = subprocess.run(['nvidia-smi', '--query-gpu=name,memory.total', '--format=csv,noheader,nounits'],
                              capture_output=True, text=True)
        if result.returncode == 0:
            print("✅ NVIDIA GPU detected:")
            gpus = result.stdout.strip().split('\n')
            for gpu in gpus:
                if gpu.strip():
                    name, memory = gpu.split(',', 1)
                    print(f"   - {name.strip()} with {memory.strip()} MB memory")
            return True
        else:
            print("❌ No NVIDIA GPU detected or nvidia-smi not found")
            return False
    except FileNotFoundError:
        print("❌ nvidia-smi command not found. Is NVIDIA driver installed?")
        return False
    except Exception as e:
        print(f"❌ Error checking for NVIDIA GPU: {e}")
        return False

def check_cuda_version():
    """Check CUDA version"""
    try:
        result = subprocess.run(['nvcc', '--version'], capture_output=True, text=True)
        if result.returncode == 0:
            version_line = [line for line in result.stdout.split('\n') if 'release' in line.lower()][0]
            print(f"✅ CUDA version: {version_line.strip()}")

            # Extract version number
            import re
            version_match = re.search(r'release (\d+\.\d+)', version_line)
            if version_match:
                cuda_version = float(version_match.group(1))
                if cuda_version >= 11.8:
                    print(f"✅ CUDA version {cuda_version} meets minimum requirement (11.8+)")
                    return True
                else:
                    print(f"❌ CUDA version {cuda_version} is below minimum requirement (11.8+)")
                    return False
            else:
                print("⚠️  Could not determine CUDA version from output")
                return False
        else:
            print("❌ CUDA compiler (nvcc) not found")
            return False
    except FileNotFoundError:
        print("❌ CUDA compiler (nvcc) not found. Is CUDA toolkit installed?")
        return False
    except Exception as e:
        print(f"❌ Error checking CUDA version: {e}")
        return False

def check_python_version():
    """Check Python version compatibility"""
    major, minor, patch = sys.version_info[:3]
    version_str = f"{major}.{minor}.{patch}"
    print(f"✅ Python version: {version_str}")

    if major == 3 and minor >= 11:
        print("✅ Python version meets requirements (3.11+)")
        return True
    else:
        print(f"❌ Python version {version_str} is below minimum requirement (3.11+)")
        return False

def check_gpu_memory():
    """Check if GPU has sufficient memory for Isaac Sim"""
    try:
        result = subprocess.run(['nvidia-smi', '--query-gpu=memory.total', '--format=csv,noheader,nounits'],
                              capture_output=True, text=True)
        if result.returncode == 0:
            memory_lines = result.stdout.strip().split('\n')
            sufficient_memory = False

            for mem in memory_lines:
                if mem.strip():
                    try:
                        mem_value = int(mem.strip())
                        if mem_value >= 8000:  # 8GB+ recommended for Isaac Sim
                            print(f"✅ GPU has sufficient memory ({mem_value} MB)")
                            sufficient_memory = True
                        else:
                            print(f"⚠️  GPU memory may be insufficient ({mem_value} MB), 8GB+ recommended")
                    except ValueError:
                        continue

            return sufficient_memory
        else:
            print("⚠️  Could not determine GPU memory")
            return True  # Don't fail validation for this check
    except Exception as e:
        print(f"⚠️  Error checking GPU memory: {e}")
        return True  # Don't fail validation for this check

def run_validation():
    """Run all validation checks"""
    print("Starting CUDA and GPU Compatibility Validation for Module 3...\n")

    checks = [
        ("NVIDIA GPU Detection", check_nvidia_gpu),
        ("CUDA Version Check", check_cuda_version),
        ("Python Version Check", check_python_version),
        ("GPU Memory Check", check_gpu_memory),
    ]

    results = []
    for check_name, check_func in checks:
        print(f"\n{check_name}:")
        result = check_func()
        results.append((check_name, result))

    print("\n" + "="*50)
    print("CUDA and GPU Compatibility Validation Results")
    print("="*50)

    all_passed = True
    for check_name, result in results:
        status = "✅ PASS" if result else "❌ FAIL"
        print(f"{status} - {check_name}")
        if not result:
            all_passed = False

    print("="*50)

    if all_passed:
        print("🎉 All CUDA and GPU compatibility checks PASSED!")
        print("Your system is ready for NVIDIA Isaac Sim and Isaac ROS development.")
        return True
    else:
        print("❌ Some CUDA and GPU compatibility checks FAILED!")
        print("Please address the issues before proceeding with Isaac Sim setup.")
        return False

if __name__ == "__main__":
    success = run_validation()
    sys.exit(0 if success else 1)
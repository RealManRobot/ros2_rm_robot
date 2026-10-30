#!/bin/bash
# Version: 1.0
# Date: $(date +%Y-%m-%d)
# Author: Your Name
# Description: Install libapi_cpp.so for target system

set -e

# Check sudo privileges
if [ "$(id -u)" != "0" ]; then
    echo "This script must be run with sudo. Example: sudo $0"
    exit 1
fi

case "$(uname -m)" in
    x86_64) sdk_dir="linux_x86_c++_v1.1.7" ;;
    aarch64|arm64) sdk_dir="linux_arm64_c++_v1.1.7" ;;
    *) echo "Error: unsupported architecture"; exit 1 ;;
esac
cd -- "$(dirname -- "$0")"
sdk_library="$sdk_dir/libapi_cpp.so"
if [ ! -f "$sdk_library" ]; then
    echo "Error: SDK library not found: $sdk_library"
    exit 1
fi
sudo cp "$sdk_library" /usr/local/lib/libapi_cpp.so
sudo cp "$sdk_library" ./libapi_cpp.so

# Configure library path
echo "Updating system library configuration..."
if ! grep -q "/usr/local/lib" /etc/ld.so.conf; then
    echo "/usr/local/lib" | sudo tee -a /etc/ld.so.conf
fi

# Update library cache
sudo /sbin/ldconfig

# Completion message
echo -e "\n\033[1;32m[SUCCESS] libapi_cpp.so installed successfully!\033[0m"

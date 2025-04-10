#!/bin/bash

# Improved Nginx Reverse Proxy Setup Script
# Handles multiple (name, port) pairs and sets up reverse proxies for each

set -e  # Exit on error

# Color codes for pretty output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Data list of target URLs and ports - modify this as needed
TARGETS=(
    "skill_viewer 8076"
    "skill_deps 8077"
    "merge_log 8075"
    # Add more targets here as "name port" pairs
)

# Function to display status messages
status() {
    local color=$1
    local message=$2
    echo -e "${color}==> ${message}${NC}"
}

# Check if running as root
if [ "$(id -u)" -ne 0 ]; then
    status $RED "This script must be run as root"
    exit 1
fi

# Install Nginx if not already installed
if ! command -v nginx &> /dev/null; then
    status $YELLOW "Installing Nginx..."
    apt update
    apt install -y nginx
    status $GREEN "Nginx installed successfully"
else
    status $GREEN "Nginx is already installed"
fi

# Process each target
for target in "${TARGETS[@]}"; do
    read -r name port <<< "$target"
    
    status $YELLOW "Processing target: $name (port $port)"
    
    # Create Nginx config file
    config_file="/etc/nginx/sites-available/$name"
    
    if [ -f "$config_file" ]; then
        status $YELLOW "Config file already exists for $name - backing up"
        mv "$config_file" "${config_file}.bak"
    fi
    
    cat > "$config_file" << EOF
server {
    listen 80;
    server_name $name;
    
    access_log /var/log/nginx/${name}.access.log;
    error_log /var/log/nginx/${name}.error.log;
    
    location / {
        proxy_pass http://127.0.0.1:${port};
        proxy_set_header Host \$host;
        proxy_set_header X-Real-IP \$remote_addr;
        proxy_set_header X-Forwarded-For \$proxy_add_x_forwarded_for;
        proxy_set_header X-Forwarded-Proto \$scheme;
    }
}
EOF
    
    status $GREEN "Created Nginx config for $name"
    
    # Create symlink in sites-enabled if it doesn't exist
    if [ ! -L "/etc/nginx/sites-enabled/$name" ]; then
        ln -s "$config_file" "/etc/nginx/sites-enabled/"
        status $GREEN "Created symlink for $name in sites-enabled"
    else
        status $YELLOW "Symlink already exists for $name"
    fi
    
    # Add to /etc/hosts if not already present
    if ! grep -q "127.0.0.1 $name" /etc/hosts; then
        echo "127.0.0.1 $name" >> /etc/hosts
        status $GREEN "Added $name to /etc/hosts"
    else
        status $YELLOW "$name already exists in /etc/hosts"
    fi
done

# Test Nginx configuration
status $YELLOW "Testing Nginx configuration..."
if nginx -t; then
    status $GREEN "Nginx configuration test successful"
else
    status $RED "Nginx configuration test failed"
    exit 1
fi

# Restart Nginx
status $YELLOW "Restarting Nginx..."
systemctl restart nginx

# Verify Nginx is running
if systemctl is-active --quiet nginx; then
    status $GREEN "Nginx is running successfully"
else
    status $RED "Nginx failed to start"
    exit 1
fi

# Display summary
status $GREEN "\nSetup complete! The following sites are now configured:"
for target in "${TARGETS[@]}"; do
    read -r name port <<< "$target"
    echo -e "  - ${GREEN}http://${name}${NC} (proxying to port ${port})"
done

echo -e "\nYou can access these sites locally. For external access:"
echo -e "1. Configure DNS to point these hostnames to your server's IP"
echo -e "2. Ensure your firewall allows HTTP traffic (port 80)"
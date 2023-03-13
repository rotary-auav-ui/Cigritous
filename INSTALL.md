# Installation Tutorial

```bash
# Create ROS2 workspace

mkdir cigritous_ws && cd cigritous_ws

# Clone the package

git clone --recurse-submodules -b main https://github.com/rotary-auav-ui/cigritous.git

# Rename to src

mv cigritous src

# Build the package

colcon build
```

**If submodule clone fails,**

```bash
cd src
git submodule update --init
cd ..
```

Then continue build the package again
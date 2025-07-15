#!/bin/bash
# 🎯 Demo Launcher - Choose Your Demo!

echo "🤖 UR5e Robot Demos"
echo "==================="
echo ""
echo "Choose your demo:"
echo ""
echo "1️⃣  Interactive GUI Demo"
echo "   • Manual control with sliders"
echo "   • Explore robot structure"
echo "   • Perfect for learning"
echo ""
echo "2️⃣  Automatic Circle Drawing Demo"
echo "   • Programmatic robot control"
echo "   • Smooth circular motion"
echo "   • No GUI conflicts"
echo ""
echo "Which demo would you like to run? (1 or 2): "
read choice

case $choice in
    1)
        echo ""
        echo "🎮 Starting Interactive GUI Demo..."
        python3 /workspace/robot/demos/demo1_interactive_gui.py
        ;;
    2)
        echo ""
        echo "🎨 Starting Automatic Circle Drawing Demo..."
        python3 /workspace/robot/demos/demo2_automatic_circle.py
        ;;
    *)
        echo ""
        echo "❌ Invalid choice. Please run the script again and choose 1 or 2."
        exit 1
        ;;
esac
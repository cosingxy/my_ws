#!/bin/bash



set -e  # 遇到错误立即退出

echo "======================================"
echo "  RML 63 EE Hybrid Planning 编译工具"
echo "======================================"
echo ""

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 0. 先 Source 所有依赖的工作空间
echo -e "${YELLOW}[0/4] 配置编译环境...${NC}"
source /home/gxy/ws_moveit2/install/setup.bash
echo -e "${GREEN}✓ 环境配置完成${NC}"
echo ""

# 1. 编译 rml_63_ee_description（基础包）
echo -e "${YELLOW}[1/4] 编译 rml_63_ee_description...${NC}"
cd /home/gxy/my_ws
colcon build --packages-select rml_63_ee_description --symlink-install
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ rml_63_ee_description 编译成功${NC}"
else
    echo -e "${RED}✗ rml_63_ee_description 编译失败${NC}"
    exit 1
fi
echo ""

# 2. 编译 rml_63_ee_moveit_config（依赖 description）
echo -e "${YELLOW}[2/4] 编译 rml_63_ee_moveit_config...${NC}"
cd /home/gxy/my_ws
colcon build --packages-select rml_63_ee_moveit_config --symlink-install
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ rml_63_ee_moveit_config 编译成功${NC}"
else
    echo -e "${RED}✗ rml_63_ee_moveit_config 编译失败${NC}"
    exit 1
fi
echo ""

# 3. 编译 rm63_ee_hybrid_planning
echo -e "${YELLOW}[3/4] 编译 rm63_ee_hybrid_planning...${NC}"
cd /home/gxy/my_ws
colcon build --packages-select rm63_ee_hybrid_planning --symlink-install
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ rm63_ee_hybrid_planning 编译成功${NC}"
else
    echo -e "${RED}✗ rm63_ee_hybrid_planning 编译失败${NC}"
    exit 1
fi
echo ""

# 4. Source 环境
echo -e "${YELLOW}[4/4] 配置环境变量...${NC}"
source /home/gxy/my_ws/install/setup.bash
source /home/gxy/ws_moveit2/install/setup.bash
echo -e "${GREEN}✓ 环境配置完成${NC}"
echo ""

# 4. 验证
echo -e "${YELLOW}验证包是否可用...${NC}"
if ros2 pkg list | grep -q "rml_63_ee_description"; then
    echo -e "${GREEN}✓ rml_63_ee_description 包可用${NC}"
else
    echo -e "${RED}✗ rml_63_ee_description 包不可用${NC}"
fi

if ros2 pkg list | grep -q "rml_63_ee_moveit_config"; then
    echo -e "${GREEN}✓ rml_63_ee_moveit_config 包可用${NC}"
else
    echo -e "${RED}✗ rml_63_ee_moveit_config 包不可用${NC}"
fi

if ros2 pkg list | grep -q "rm63_ee_hybrid_planning"; then
    echo -e "${GREEN}✓ rm63_ee_hybrid_planning 包可用${NC}"
else
    echo -e "${RED}✗ rm63_ee_hybrid_planning 包不可用${NC}"
fi
echo ""

# 5. 显示使用说明
echo "======================================"
echo -e "${GREEN}  编译完成！${NC}"
echo "======================================"
echo ""
echo "运行 Hybrid Planning Demo："
echo ""
echo "  方法1 - 查看可用 launch 文件："
echo "    ls /home/gxy/my_ws/src/rm63_ee_hybrid_planning/test/launch/"
echo ""
echo "  方法2 - 启动组件："
echo "    # Terminal 1:"
echo "    ros2 launch rm63_ee_hybrid_planning hybrid_planning_demo.launch.py"
echo ""
echo "    # Terminal 2:"
echo "    ros2 run rm63_ee_hybrid_planning hybrid_planning_demo_node \\"
echo "      --ros-args -p hybrid_planning_action_name:=/hybrid_planning/run_hybrid_planning"
echo ""
echo "  方法3 - 手动运行 demo 节点："
echo "    cd /home/gxy/my_ws"
echo "    source install/setup.bash"
echo "    ./install/rm63_ee_hybrid_planning/lib/rm63_ee_hybrid_planning/hybrid_planning_demo_node \\"
echo "      --ros-args -p hybrid_planning_action_name:=/hybrid_planning/run_hybrid_planning"
echo ""
echo -e "${YELLOW}📚 查看完整文档：${NC}"
echo "    cat /home/gxy/my_ws/HYBRID_PLANNING_RML63_GUIDE.md"
echo ""
echo "======================================"

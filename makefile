


# 项目配置
PROJECT = 全场定位测试

# 包含 VEX 标准构建配置
include vex/mkenv.mk

# 源文件列表
SOURCES_C = 
SOURCES_CPP = \
    src/main.cpp \
    src/robot-config.cpp \
    src/Move_Motor.cpp \
    src/PID.cpp \
    src/Auto.cpp \
    src/map.cpp \
    src/autons.cpp \
    src/FieldMap.cpp \
    src/KalmanFilter.cpp \
    src/Localization.cpp

# 包含目录
INCLUDES = -Iinclude

# 添加到构建系统
SRC_C += $(SOURCES_C)
SRC_CPP += $(SOURCES_CPP)

# 添加编译标志
CFLAGS += $(INCLUDES)
CXXFLAGS += $(INCLUDES)

# 包含构建规则
include vex/mkrules.mk

# 显示构建信息
info:
	@echo "Building project: $(PROJECT)"
	@echo "Using VEX build system from: vex/"

.PHONY: info
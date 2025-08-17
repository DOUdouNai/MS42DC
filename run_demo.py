#!/usr/bin/env python3
"""
脊柱姿态可视化系统 - 快速启动脚本
"""

import sys
import os

def print_menu():
    """打印菜单"""
    print("=" * 50)
    print("脊柱姿态可视化系统")
    print("=" * 50)
    print("1. 运行演示程序 (推荐)")
    print("2. 运行实时可视化 (需要串口数据)")
    print("3. 运行基础可视化")
    print("4. 生成测试数据")
    print("5. 退出")
    print("=" * 50)

def run_program(program_name):
    """运行指定的程序"""
    try:
        if program_name == "demo":
            os.system("python spine_demo.py")
        elif program_name == "realtime":
            os.system("python spine_realtime.py")
        elif program_name == "basic":
            os.system("python spine_visualization.py")
        elif program_name == "test_data":
            os.system("python test_data_generator.py")
    except Exception as e:
        print(f"运行程序时出错: {e}")

def main():
    """主函数"""
    while True:
        print_menu()
        
        try:
            choice = input("请选择要运行的程序 (1-5): ").strip()
            
            if choice == "1":
                print("启动演示程序...")
                run_program("demo")
            elif choice == "2":
                print("启动实时可视化...")
                print("注意：需要COM14端口有IMU数据")
                run_program("realtime")
            elif choice == "3":
                print("启动基础可视化...")
                run_program("basic")
            elif choice == "4":
                print("启动测试数据生成器...")
                run_program("test_data")
            elif choice == "5":
                print("退出程序")
                break
            else:
                print("无效选择，请重新输入")
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
            break
        except Exception as e:
            print(f"发生错误: {e}")

if __name__ == "__main__":
    main()

import time
import traceback

import sys
from datetime import datetime

class BugLogger:
    """Bug日志记录器"""
    
    def __init__(self, log_file="bug_log.txt"):
        """初始化日志文件"""
        self.log_file = log_file
        # 初始化时写入一条分隔线
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(f"\n{'='*50}\n")
            f.write(f"程序启动时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
            f.write(f"{'='*50}\n")
    
    def log_bug(self, error_message=None):
        """
        记录bug到日志文件
        可以手动传入错误信息，或者自动捕获当前异常
        """
        timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
        
        with open(self.log_file, 'a', encoding='utf-8') as f:
            f.write(f"\n【Bug记录】时间: {timestamp}\n")
            
            if error_message:
                # 手动传入的错误信息
                f.write(f"错误信息: {error_message}\n")
            else:
                # 自动捕获当前异常
                exc_type, exc_value, exc_traceback = sys.exc_info()
                if exc_value:
                    f.write(f"错误类型: {exc_type.__name__}\n")
                    f.write(f"错误信息: {exc_value}\n")
                    f.write("详细追踪:\n")
                    traceback.print_exc(file=f)
                else:
                    f.write("没有检测到异常\n")
            
            f.write("-" * 30 + "\n")
    
    def view_log(self):
        """查看日志文件内容"""
        try:
            with open(self.log_file, 'r', encoding='utf-8') as f:
                print(f.read())
        except FileNotFoundError:
            print("日志文件不存在,还没有记录任何bug")

# 创建一个全局日志记录器实例，方便直接使用
logger = BugLogger()

# 为了方便使用，定义几个函数

def log_bug(error_message=None):
    """快速记录bug的函数"""
    logger.log_bug(error_message)

def view_log():
    """快速查看日志的函数"""
    logger.view_log()

# 如果直接运行这个文件，测试一下
if __name__ == "__main__":
    print("Bug日志模块测试:")
    
    # 测试1:手动记录
    log_bug("测试手动记录的错误")
    
    # 测试2:自动捕获异常
    try:
        x = 1 / 0  # 故意制造错误
    except:
        log_bug()
    
    print(f"日志已记录到: bug_log.txt")
    print("查看日志内容:")
    view_log()
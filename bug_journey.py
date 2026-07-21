import time
from datetime import datetime

def record_time():
    now = datetime.now()#这一步并不是必须的，但是是规范写法
    return now.strftime('%Y-%m-%d %H:%M:%S')
def save_error (error_message):
    time_str = record_time()
    with open('bug_log.txt','a',encoding= 'UTF-8') as f:
        f.write(f"[{time_str}]{error_message}\n")
# bug_journey.py - Bug日志记录模块
def view_log():
    with open ("bug_log.txt","r",encoding='UTF-8') as f:
        print(f.read())
def auto_log(func):
    def wrapper(*args,**kwargs):
        try:
            return func(*args,**kwargs)
        except Exception as e:
            save_error(f'{func.__name__}出错：{str(e)}')
            print(f'错误已记录')
    return wrapper   




if __name__== '__main__':
    print(f'{record_time()}')

    try:
        result = 10/0
    except Exception as e:
        save_error(str(e))#将错误变成字符串写入到buglog文件里面
    view_log()
         

    
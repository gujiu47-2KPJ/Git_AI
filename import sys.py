

import time
"""def c(age):
    if age > 18:
       return 
         
    else :age < 18
    return False
result = c(17)
re = 111
print(111)
def judage( ):
   if  result is False:
    print("yes")
name =['a','b','c','d']
for i in range(3):
   print(name[i])
for i in range(3):
   print(name[-i ]) 
mylist = [1,2,3,3]

mylist[1] = 0
index = mylist.index(0)
print(f"{index}")
mylist.pop(1)
index = mylist.index(3)
print(f"{index}")
mylist.insert(0,3)
index =  mylist.index(3)
print(f"{index}")
del mylist[0]
mylist.insert(0,2)
print(f"{index}")
index = mylist.index(2)
print(f"{index}")
str = 'i is the god'
Value = str[1]
print(Value)
new = str.split()
print(new)
newstr = str.strip('is')
print(newstr)
集合= {'str','you','point'}
lens = len(集合)
print(f'{lens}')"""

def print1():
   return 1,2
x,y = print1()
print(f'{x},{y}')


def user (*args):
   print(args)

user('tom')

with open(r'F:\python-pratice\change_first_letter.py', 'r+', encoding='UTF-8') as f :

    p = f.read()
   #print (f'{p}')
   #time.sleep(10)

    f.write('hello wrold')


    time.sleep(2)

    f.flush()

    f.seek(0)

    p1 = f.read() 

    print(f'{p1}')

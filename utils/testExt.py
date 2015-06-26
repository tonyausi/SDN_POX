def foo(strin):
    print strin
    
class A():
    bar = staticmethod(foo)
    
def an (n, **kw):
  kw['label'] = str(n)
  return {'an':{str(n):kw}}
    
if __name__ == "__main__":
    foo('me')
    test = A()
    test.bar('you')
    print an(1,kind='switch'), an(1)
    

class Animal(object):
    def eat(self):
        print("I eat all")

class C(object):
    def eat(self):
        print("I too eat")

class Wolf(C, Animal):
    def eat(self):
        print("I am Non Veg")
        super(Wolf, self).eat()
        Animal.eat(self)

w = Wolf()
w.eat()

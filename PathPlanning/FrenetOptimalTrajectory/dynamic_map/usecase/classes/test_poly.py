from poly import Poly

# print("----- func1 -----")
poly = Poly(1)
xl= list(range(0, 10))
def func1(x):
    return x + 1

for x in xl:
    poly.add_constrain(func1(x), x, 0)
poly.set_coefficients()

for x in xl:
    print(f"{func1(x)}, {poly.value(x)}, {poly.value(x, 1)}")

# print("----- func2 -----")
poly = Poly(1)
xl= list(range(0, 10))

def func2(x):
    return 2 * x + 5

for x in xl:
    poly.add_constrain(func2(x), x, 0)
poly.set_coefficients()

for x in xl:
    print(f"{func2(x)}, {poly.value(x)}, {poly.value(x, 1)}")


# print("----- func3 -----")
poly = Poly(2)
xl= list(range(0, 10))

def func3(x):
    return 5 + 2 * x + 3 * x**2

for x in xl:
    poly.add_constrain(func3(x), x, 0)

poly.set_coefficients()

for x in xl:
    print(f"{func3(x)}, {poly.value(x)}, {poly.value(x, 1)}, {poly.value(x, 2)}")


# print("----- func4 derivation -----")
poly = Poly(3)
xl= list(range(0, 10))

def func4(x):
    return 1 + x + x**2 + x**3

poly.add_constrain(4, 1, 0)
poly.add_constrain(6, 1, 1)
poly.add_constrain(8, 1, 2)
poly.add_constrain(6, 1, 3)
poly.set_coefficients()

print("1 + x + x^2 + x^3")
for x in xl:
    print(f"{func4(x)}, {poly.value(x)}, {poly.value(x, 1)}, {poly.value(x, 2)}, {poly.value(x, 3)}")

# print("----- func5 derivation -----")
poly = Poly(3)
xl= list(range(0, 10))

def func5(x):
    return 1 + 3 * x + 5 * x**2 + 7 * x**3

poly.add_constrain(16, 1, 0)
poly.add_constrain(1, 0, 0)
# 3 + 10x + 21x^2
poly.add_constrain(34, 1, 1)
# 10 + 42x
poly.add_constrain(10, 0, 2)
poly.set_coefficients()

for x in xl:
    print(f"{func5(x)}, {poly.value(x)}, {poly.value(x, 1)}, {poly.value(x, 2)}, {poly.value(x, 3)}")

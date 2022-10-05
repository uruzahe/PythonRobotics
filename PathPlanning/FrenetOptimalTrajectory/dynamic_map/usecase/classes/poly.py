import numpy as np
import sys
class Poly:
    def __init__(self, dim):
        self.A = [1 for i in range(0, dim + 1)]
        self.X = []
        self.Y = []

    def value(self, x, derivation=0):
        # self.set_coefficients()

        ks = [self.factorial(i, derivation) for i in range(0, len(self.A))]
        al = list(self.A)

        return sum([al[i] * ks[i] * x ** max([0, (i - derivation)]) for i in range(0, len(self.A))])

    def add_constrain(self, val, x, derivation=0, ks=None):
        ks = [self.factorial(i, derivation) for i in range(0, len(self.A))]

        # # print(ks)
        self.Y.append(val)
        self.X.append([ks[i] * x ** max([0, (i - derivation)]) for i in range(0, len(self.A))])

    def set_coefficients(self):
        X = np.array(self.X)
        Y = np.array(self.Y)

        # print(np.dot(X.T, X))
        try:
            self.A = np.dot(np.linalg.inv(np.dot(X.T, X)), np.dot(X.T, Y.T))
            # return True

        except Exception:
            # return False
            print("bb")
            self.A = np.dot(np.linalg.pinv(np.dot(X.T, X)), np.dot(X.T, Y.T))
            # sys.exit()
        # print(self.A)

    def factorial(self, val, i):
        if i <= 0:
            return 1

        else:
            return val * self.factorial(val - 1, i - 1)

    def calc_point(self, x):
        return self.value(x, 0)

    def calc_first_derivative(self, x):
        return self.value(x, 1)

    def calc_second_derivative(self, x):
        return self.value(x, 2)

    def calc_third_derivative(self, x):
        return self.value(x, 3)

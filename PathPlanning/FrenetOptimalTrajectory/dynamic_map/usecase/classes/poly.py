import numpy as np

class Poly:
    def __init__(self, dim):
        self.A = [1 for i in range(0, dim + 1)]
        self.X = []
        self.Y = []

    def value(self, x, derivation=0):
        self.set_coefficients()

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
        self.A = np.dot(np.linalg.inv(np.dot(X.T, X)), np.dot(X.T, Y.T))

    def factorial(self, val, i):
        if i <= 0:
            return 1

        else:
            return val * self.factorial(val - 1, i - 1)

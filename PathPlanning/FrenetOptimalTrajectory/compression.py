# author Tarun Bisht
import math
import sys

from iteration_utilities import deepflatten

sys.setrecursionlimit(10000)

class Symbol:
    def __init__(self, symbol):
        self.symbol = symbol
        self.code = ""
        self.__prob = 0

    def prob(self):
        return self.__prob

    def add_prob(self, prob):
        self.__prob = prob

        return self

    def myself(self):
        return self

    def nodes(self):
        return self

class HuffmanNode:
    def __init__(self, left, right):
        self.left = left
        self.right = right

    def prob(self):
        p1 = 0 if self.left is None else self.left.prob()
        p2 = 0 if self.right is None else self.right.prob()

        return p1 + p2

    def add_code(self, prev_code):
        index = 0

        for node in [self.left, self.right]:
            if isinstance(node, Symbol):
                node.code = prev_code + str(index)

            elif isinstance(node, HuffmanNode):
                node.add_code(prev_code + str(index))

            else:
                Exception("unknown type")

            index = index + 1

    def nodes(self):
        return [self.left.nodes(), self.right.nodes()]

class BaseCompression:
    def __init__(self):
        self.result = None
        # self.code2symbol = None

    def symbols(self, data, ascending=False):
        result = [Symbol(d).add_prob(data.count(d) / len(data)) for d in list(set(data))]

        return sorted(result, key=lambda x: x.prob(), reverse=ascending)

    def compress(self):
        Exception("This method should be overrided.")

    def decompress(self, bit_str, bit_length):
        # if self.code2symbol == None:
        code2symbol = {d.code: "{0:b}".format(int(d.symbol)).zfill(bit_length) for d in self.result}
        # print(f"{bit_str}")
        # print(code2symbol)

        result = ""
        while 0 < len(bit_str):
            origin_bit_str = bit_str[:]

            for i in range(1, len(bit_str) + 1):
                if bit_str[0:i] in code2symbol.keys():
                    result = result + code2symbol[bit_str[0:i]]
                    # print(f"{bit_str[0:i]}, {code2symbol[bit_str[0:i]]}")
                    bit_str = bit_str[i:]
                    # print(bit_str)
                    break

                else:
                    continue

            assert(bit_str != origin_bit_str)
        return result

class ShannonFennonCompression(BaseCompression):
    def compress(self, data):
        symbols = self.symbols(data, ascending=True)

        self.encode(symbols)
        self.result = symbols

        return self.result


    def split_index(self, symbols, index):
        diff_1 = math.fabs(sum([s.prob() for s in symbols[:index]]) - sum([s.prob() for s in symbols[index:]]))
        diff_2 = math.fabs(sum([s.prob() for s in symbols[:index + 1]]) - sum([s.prob() for s in symbols[index + 1:]]))

        if diff_1 <= diff_2:
            return index

        else:
            return self.split_index(symbols, index+1)


    def encode(self, symbols):
        split_index = self.split_index(symbols, 0)

        if split_index == 0:
            return

        part_1 = symbols[:split_index]
        part_2 = symbols[split_index:]

        for i in part_1:
            i.code += '0'

        for i in part_2:
            i.code += '1'

        self.encode(part_1)
        self.encode(part_2)


class HuffmanCompression(BaseCompression):
    def compress(self, data):
        symbols = self.symbols(data, ascending=False)

        while 2 <= len(symbols):
            c1 = symbols.pop(0)
            c2 = symbols.pop(0)

            symbols.append(HuffmanNode(c1, c2))
            symbols = sorted(symbols, key=lambda x: x.prob(), reverse=False)

        if isinstance(symbols[0], Symbol):
            symbols[0].code = "0"

        elif isinstance(symbols[0], HuffmanNode):
            symbols[0].add_code("")

        else:
            Exception("unknown type")

        self.result = list(deepflatten([symbols[0].nodes()]))
        return self.result


if __name__ == '__main__':
    texts = ["shannon_fennon", "huffman", "ABAAACBDBAB"]

    print("----- channon fennon compression -----")
    comp = ShannonFennonCompression()
    for t in texts:
        print(f"--- text: {t} ---")
        symbols = comp.compress(t)

        for s in sorted(symbols, key=lambda x: x.prob(), reverse=True):
            print(f"{s.symbol}: {s.prob()}, {s.code}")

    print("----- huffman compression -----")
    comp = HuffmanCompression()
    for t in texts:
        print(f"--- text: {t} ---")
        symbols = comp.compress(t)

        for s in sorted(symbols, key=lambda x: x.prob(), reverse=True):
            print(f"{s.symbol}: {s.prob()}, {s.code}")

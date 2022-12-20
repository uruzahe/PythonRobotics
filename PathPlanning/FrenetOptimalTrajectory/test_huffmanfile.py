import huffmanfile
hfc = huffmanfile.HuffmanCompressor()
out1 = hfc.compress(b"Some data aaaaa")
out2 = hfc.compress(b"Another piece of data\n")
out3 = hfc.compress(b"Even more data\n")
out4 = hfc.flush()
# Concatenate all the partial results:
result = b"".join([out1, out2, out3, out4])
print(result)

decomp = huffmanfile.HuffmanDecompressor()
print(decomp.decompress(result))

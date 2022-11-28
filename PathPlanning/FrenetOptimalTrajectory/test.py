import time
from concurrent.futures.process import ProcessPoolExecutor

def wait_on_b():
    time.sleep(5)
    print(b.result())  # b will never complete because it is waiting on a.
    return 5

def wait_on_a():
    time.sleep(5)
    print(a.result())  # a will never complete because it is waiting on b.
    return 6


# executor = ProcessPoolExecutor(max_workers=2)
with ProcessPoolExecutor(max_workers=2) as executor:
    a = executor.submit(wait_on_b)
    b = executor.submit(wait_on_a)

    print(a.result())
    print(b.result())

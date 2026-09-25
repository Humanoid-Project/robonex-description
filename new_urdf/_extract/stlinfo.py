import struct, sys, numpy as np
def load(p):
    b = open(p, "rb").read()
    n = struct.unpack("<I", b[80:84])[0]
    if 84 + 50*n == len(b):
        a = np.frombuffer(b[84:], dtype=np.dtype([("n","<f4",3),("v","<f4",(3,3)),("a","<u2")]), count=n)
        return a["v"].reshape(-1,3).astype(float), n
    vs = [list(map(float, l.split()[1:4])) for l in b.decode(errors="ignore").splitlines() if l.strip().startswith("vertex")]
    return np.array(vs), len(vs)//3
for p in sys.argv[1:]:
    v, n = load(p)
    print("%-40s tris=%6d min=%s max=%s" % (p.split("/")[-1], n, np.round(v.min(0), 2), np.round(v.max(0), 2)))

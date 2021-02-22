def rk2(a, t, h, vi):
    return vi + a * h

def rk2_x(a, t, h, v, x):
    return x + h * v + a / 2 * h * h


def main():
    h = 0.01
    time = 0
    vels = []
    xs = []
    vi = 0
    xi = 0
    a = 1
    for i in range(200):
        if i >= 150:
            a = -1
        vnext = rk2(a, time, h, vi)
        xnext = rk2_x(a, time, h, vi, xi)
        xs.append(xnext)
        vels.append(vnext)
        vi = vnext
        xi = xnext
        time += h
    print(vels)
    print(xs)
    print(time)
    print(a)

if __name__ == '__main__':
    main()



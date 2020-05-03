#!/usr/bin/env python
import math

U = 2
L = 3


def solve_sss(a, b, c, trace=False):
    # given side lengths a,b,c returns angles A,B,C (angle is opposit to side
    #
    # see https://www.mathsisfun.com/algebra/trig-solving-sss-triangles.html

    a = float(a)
    b = float(b)
    c = float(c)
    if trace: print("a:", a, "b:", b, "c:", c)
    A = math.acos((b*b + c*c - a*a)/(2* b*c))
    B = math.acos((c*c + a*a - b*b)/(2*c*a))
    C = math.pi - A - B
    if trace: print("A:", A*180./math.pi, "B:", B*180./math.pi, "C:", C*180./math.pi)

    return (A, B, C)

def get_joint_angles_for_position(x, y, U, L, trace=False):
    # solve joint angles for point x,y with link lengths U and L
    # assumes joints are straight down at angle of zero
    # returns angles (u, l)
    x=float(x)
    y=float(y)
    U=float(U)
    L=float(L)

    if trace:
        print("x:",x,"y:",y,"U:",U,"L:",L)

    length = math.sqrt(x*x+y*y)
    theta = math.atan2(y,x)
    (alpha, beta, _) = solve_sss(L, length, U, trace)
    u = theta + alpha + math.pi/2
    l = beta - math.pi

    if trace:
        print("u:", u*180./math.pi, "l:", l*180./math.pi)
        print('')
    return (u, l)

def test_sss(a, b, c, trace=False):
    print("a:", a, "b:", b, "c:", c)
    (A, B, C) = solve_sss(a, b, c)
    print("A:", A*180./math.pi, "B:", B*180./math.pi, "C:", C*180./math.pi)

def test_get_joint_angles_for_position(x, y, U, L):
    print("x:", x, "y:", y, "U:", U, "L:", L)
    (u,l) = get_joint_angles_for_position(x, y, U, L)
    print("u:", u*180./math.pi, "l:", l*180./math.pi)
    print('')


def main():
    solve_sss(1, 1, 1, trace=True)
    solve_sss(3, 4, 5, trace=True)
    get_joint_angles_for_position(0, -4, 2, 2, trace=True)
    get_joint_angles_for_position(4, 0, 2, 2, trace=True)
    get_joint_angles_for_position(2, -2, 2, 2, trace=True)

if __name__ == '__main__':
    try:
        main()
    except:
        pass

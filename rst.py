import numpy
import math

class RST:
    def poly_degree(self, polynom):
        out = len(polynom)-1
        return out

    def cal_pol(self, Tr, delta, Ts):
        # Calculates a denominator polynomial for a transfer function
        # with a risetime of Tr and a damping delta(<1) for the sampling time Ts
        wo = (1/Tr)*math.exp(delta/math.sqrt(1-math.pow(delta, 2)*math.acos(delta)))
        out = [1, (-2*math.exp(-delta*wo*Ts)*math.cos(wo*math.sqrt(1-math.pow(delta, 2))*Ts)), math.exp(-2*delta*wo*Ts)]
        return out

    def poly_add(self, X, Y):
        n_x = len(X)
        n_y = len(Y)
        # make vectors equally long by adding zeros to the end
        n = max(n_x, n_y)
        X = numpy.append(numpy.zeros((n-n_x)), X)
        Y = numpy.append(numpy.zeros((n-n_y)), Y)
        Z = numpy.add(X,Y)
        # remove possible leading zeros
        Z = numpy.trim_zeros(Z, 'f')
        return Z

    def rst_design(self, Bminus, Bplus, Aminus, Aplus, Rd, Sd, Acl_dash, Ao):
        A = numpy.convolve(Aplus, Aminus)
        B = numpy.convolve(Bplus, Bminus)
        # extended plant
        Adash = numpy.convolve(Aminus, Rd)
        Bdash = numpy.convolve(Bminus, Sd)
        # double check degree
        deg_Acl = 2*self.poly_degree(A)-1+self.poly_degree(Rd)+self.poly_degree(Sd)
        deg_Acl_dash = deg_Acl-self.poly_degree(Aplus)-self.poly_degree(Bplus)
        if deg_Acl_dash != self.poly_degree(Acl_dash):
            print("Error: wrong degree of Acl_dash")
            return

        deg_Sdash = self.poly_degree(Aminus) + self.poly_degree(Rd) - 1
        deg_Rdash = deg_Sdash + self.poly_degree(Sd) + self.poly_degree(Aplus) - self.poly_degree(Rd) - self.poly_degree(Bplus)

        deg_S = deg_Sdash + self.poly_degree(Sd) + self.poly_degree(Aplus)
        if deg_S != self.poly_degree(Ao):
            print("Error: deg(S) != deg(Ao)")
            return

        # check if Ao is a factor of Acl_dash*Aplus*Bplus and determine Ao
        Acr = numpy.polydiv(numpy.convolve(numpy.convolve(Acl_dash, Aplus), Bplus), Ao)
        Ac = Acr[0]
        r = Acr[1]
        if abs(sum(r)) > 1e-10:
            print("Error: r > 1e-10")

        # now determine the controller polynomials
        na = len(Adash) - 1
        nb = len(Bdash) - 1

        n_poly = max(na, nb)

        # make A and B to order n by adding zeros

        AA = numpy.append(numpy.zeros(n_poly-na), Adash)
        BB = numpy.append(numpy.zeros(n_poly-nb), Bdash)

        phi = numpy.zeros((deg_Acl_dash + 1, deg_Rdash + deg_Sdash + 2))

        for i in range(deg_Rdash+1, 0, -1):
            start = deg_Acl_dash+1+i-deg_Rdash-1-n_poly
            stop = deg_Acl_dash+1+i-deg_Rdash-1
            spaced = abs(start - stop) + 1
            places = numpy.linspace(start, stop, spaced)
            non_valid_places = numpy.nonzero(places < 1)
            places = numpy.delete(places, non_valid_places)
            k = len(places)
            counter = 0
            for pos in range(int(places[0])-1, int(places[-1])):
                phi[pos, i - 1] = numpy.transpose(AA[len(AA)-k:len(AA)])[counter]
                counter = counter + 1

        for i in range(deg_Sdash+1, 0, -1):
            start = deg_Acl_dash+1+i-deg_Sdash-1-n_poly
            stop = deg_Acl_dash+1+i-deg_Sdash-1
            spaced = abs(start - stop) + 1
            places = numpy.linspace(start, stop, spaced)
            non_valid_places = numpy.nonzero(places < 1)
            k = len(places)
            counter = 0
            for pos in range(int(places[0])-1, int(places[-1])):
                phi[pos, i + deg_Rdash] = numpy.transpose(BB[len(BB)-k:len(BB)])[counter]
                counter = counter + 1

        SOL = numpy.linalg.lstsq(phi, numpy.transpose(Acl_dash))
        SOL = SOL[0]
        Rdash = numpy.transpose(SOL[0:deg_Rdash+1])
        Sdash = numpy.transpose(SOL[deg_Rdash+1:deg_Rdash+deg_Sdash+2])

        R = numpy.convolve(numpy.convolve(Rdash, Rd), Bplus)
        S = numpy.convolve(numpy.convolve(Sdash, Sd), Aplus)

        T = numpy.divide(numpy.multiply(Ao, sum(Ac)), sum(B))

        return R, S, T

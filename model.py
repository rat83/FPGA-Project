import matplotlib.pyplot as plt


dt = (1./256)
x = [-1.]
y = [0.1]
z = [25.]
sigma = 10.0
beta = 8./3.
rho = 28.0

def dx(sigma, x, y):
    if (abs(dt*(y-x)) > 63):
        print("Overflowed x")
    return (dt*(y-x)) * sigma

def dy(rho, x, y, z):
    if (abs(x*(dt*(rho-z))) > 63):
        print("Overflowed y")
    return x*(dt)*(rho-z)-dt*y

def dz(beta, x, y, z):
    if ((abs(x*(dt*y)) > 63 ) | (abs((dt*beta)*z) > 63) | (abs(x*(dt*y) - (dt*beta)*z) > 63)):
        print("Overflowed z")
    return x*(dt*y) - (dt*beta)*z

for i in range(10000):
    x.extend([x[i] + dx(sigma, x[i], y[i])])
    y.extend([y[i] + dy(rho, x[i], y[i], z[i])])
    z.extend([z[i] + dz(beta, x[i], y[i], z[i])])
    

    
plt.plot(x)
plt.plot(y)
plt.plot(z)

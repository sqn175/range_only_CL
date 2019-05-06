function omega = Omega(w)
    omega = [ 0    -w(x) -w(y) -w(z);
              w(x)  0     w(z) -w(y);
              w(y) -w(z)  0     w(x);
              w(z)  w(y) -w(x)  0   ];
end
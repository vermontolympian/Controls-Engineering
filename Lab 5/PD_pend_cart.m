function u = PD_pend_cart(x,xd)
    kpp = -25;
    kpc = -.25;
    kdp = -2;
    kdc = -1;
    
    e_p = xd(1) - x(1);
    e_dot_p = xd(2) - x(2);
    e_c = xd(3) - x(3);
    e_dot_c = xd(4) - x(4);
    
    Upend = (kpp * e_p) + (kdp * e_dot_p);
    Ucart = (kpc * e_c) + (kdc * e_dot_c);
    
    u = Upend + Ucart;
end
function skew4x4=skew4x4(v)
skew4x4=[0 v(3) -v(2) v(1);
         -v(3) 0 v(1) v(2);
         v(2) -v(1) 0 v(3);
         -v(1) -v(2) -v(3) 0];
    

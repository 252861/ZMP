clear all;
close all;
clc;


%==========ode
    %----dane
               T = 50;
            czas = [0,T];
        war_pocz = [1,8,(pi/2),0,0];
        %opcje    = odeset('RelTol', 1e-10);

%[T,X] = ode45(@monof,czas,war_pocz,opcje)
[T,X] = ode45(@monof,czas,war_pocz);


%==========obliczenia
          x = X(:,1);
          y = X(:,2);
      theta = X(:,3);
          v = X(:,4);
          w = X(:,5);
      
          R = 1;
        w_d = 0.5;
        v_d = R*w_d;
        x_d =  R*sin(w_d*T);
        y_d = -R*cos(w_d*T);
    theta_d = w_d*T;

        x_e = x - x_d;
        y_e = y - y_d;
    theta_e = theta - theta_d;


%==========wykresy
figure(1)
hold on;
grid on;

subplot(1,3,1);
plot(T,x);
title('x(t)');

subplot(1,3,2);
plot(T,y);
title('y(t)');

subplot(1,3,3);
plot(T,theta);
title('theta(t)');


figure(2)
hold on;
grid on;

plot(x,y);
title('xy');

figure(3)
hold on;
grid on;

subplot(1,3,1);
plot(T,x_e);
title('x_e');

subplot(1,3,2);
plot(T,y_e);
title('y_e');

subplot(1,3,3);
plot(T,theta_e);
title('theta_e');

%test
figure(4)
hold on;
grid on;

subplot(1,2,1)
plot(T,v);
title('v');

subplot(1,2,2)
plot(T,w);
title('w');








%===========funkcja

function [obiekt] = monof (t,X) 
    %----zmienne
        x = X(1,1);
        y = X(2,1);
    theta = X(3,1);
%         v = X(4,1);
%         w = X(5,1);
%       eta = [v ; w];
%     
%         
%       
%     %----generator trajektori
%           R = 1;
%         w_d = 0.5;
%         
%         v_d = R*w_d;
%         
%         x_d =  R*sin(w_d*t);
%         y_d = -R*cos(w_d*t);
%     theta_d = w_d*t;
    
%     %----sterownik kinematyczny
%             %________błędy podstawowe
%                 x_e = x - x_d;
%                 y_e = y - y_d;
%             theta_e = theta - theta_d;
%             
%             %________błędy referencyjne
%                 e_x =  cos(theta)*x_e + sin(theta)*y_e;
%                 e_y = -sin(theta)*x_e + cos(theta)*y_e;
%             e_theta =  theta_e;
%             
%                 e_x_k = w*e_y + v -v_d*cos(e_theta);
%                 e_y_k = -w*e_x + v_d*sin(e_theta);
%             e_theta_k = w - w_d;
%     
%     k1 = 1;
%     k2 = 1;
%     
%     %v_ref = -k1*e_x + v_d*cos(e_theta);
%     %w_ref = -k2*e_theta + w_d - (e_y/e_theta)*v_d*sin(e_theta);
%     
%     v_ref_k = -k1*e_x_k + v_d * (-sin(e_theta)*e_theta_k);
%     w_ref_k = -k2*e_theta_k - ((e_y_k*v_d*sin(e_theta)+e_y*v_d*cos(e_theta)*e_theta_k)*e_theta - (e_y*v_d*sin(e_theta))*e_theta_k)/((e_theta)^2);
%     
% %       eta_ref = [v_ref ; w_ref];
% %     eta_ref_k = [v_ref_k ; w_ref_k];
%     
%     
%     %----sterownik dynamiczny
%             %________zmienne (nazwa Xk oznacza X z indeksem k, a nazwa X_k oznacza X z kropką)
%             Mp  = 99;
%             Mk  = 0.5; 
%             M11 = 107.9;
%             M22 = 83.6;
% 
%             L   = 0.3;
%             d   = 0;
%             Rk  = 0.075;
%             I2  = 6.609;
%             
%             Km  = [1 , 0 ; 0 , 1];
%             
%             %________wzory i definicje
%             Mc = Mp + 2*Mk; 
%             M_gwiazdka = [M11 , 0 ; 0 , M22]; 
%             
%             u = M_gwiazdka * (eta_ref_k - Km*(eta - eta_ref)); %algorytm dokładnej linearyzacji        
%             eta_k = inv(M_gwiazdka) * u;
            
    %----lab3
        %___T
        xt1 = theta;
        xt2 = -x*cos(theta)-y*sin(theta);
        xt3 = -x*sin(theta)+y*cos(theta);
        
        %___Spiżarnia - algorytm Astolfiego
        k_pom = 1;
        p2    = -5;
        p3    = 9;
        
        u1    = -k_pom*xt1;
        u2    = p2*xt2 + p3*(xt3/xt1);
        
        %___T^-1
        w = u1;
        v = -w*xt3 - u2;

    
    %----obiekt
        x_k = cos(theta)*v;
        y_k = sin(theta)*v;
    theta_k = w;
%         v_k = eta_k(1,1);
%         w_k = eta_k(2,1);
            
     obiekt = [x_k ; y_k ; theta_k ; 0; 0];
    
    
end

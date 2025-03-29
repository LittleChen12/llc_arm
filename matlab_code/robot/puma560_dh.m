function [alpha,a,d,qlim1,qlim2,qlim3,qlim4,qlim5,qlim6] = puma560_dh()

    alpha(1)=0;     a(1)=0;      d(1)=0.212;     
    alpha(2)=-pi/2; a(2)=0.04;   d(2)=0.062; 
    alpha(3)=0;     a(3)=0.18;   d(3)=-0.05;     
    alpha(4)=-pi/2; a(4)=0.055;  d(4)=0.205027;  
    alpha(5)=pi/2;  a(5)=0;      d(5)=0;     
    alpha(6)=-pi/2; a(6)=0;      d(6)=0;     

    qlim1=[-160, 160]/180*pi;
    qlim2=[-90, 90]/180*pi;
    qlim3=[-240, 90]/180*pi;
    qlim4=[-110, 170]/180*pi;
    qlim5=[-100, 100]/180*pi;
    qlim6=[-266, 266]/180*pi;

end
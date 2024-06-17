clc, clearvars 

%% Suspension parameters 

trackwidth = 1.24; %m 

Kphi_front = 580; %Front roll stiffness(nm/deg)

Kphi_rear = 555; 

hG = .35; %CG height(m)

hsF = .35; %front CG height (m)

hsR = .35; %rear CG height(m)

m_T = 300; %Total vehicle mass(kg)

WD = .54; %Weight distribution relative to front

m_usF = 10; %Front unsprung mass(kg) (left and right combined)

m_usR = 10; %Rear unsprung mass(kg)

Zf = .02785; %front roll center height(m)

Zr = .03556; %rear roll center height

RL =.205; %Tire loaded radius


%% Parameter calcs
dsf = hsF - Zf; %front roll moment arm (m)

dsr = hsR - Zr; 

lambda = Kphi_front / (Kphi_front + Kphi_rear); %Theoretical LLTD

m_sF = (m_T*(WD)) - m_usF;

m_sR = (m_T * (1-WD)) - m_usR;

%% vary chassis stiffness
for (i = 1 : +1: 100)%sweep for torsional chassis stiffnesses in increments of 100 (nm/deg)

Kch = i*100;

mu = Kch/(Kphi_front + Kphi_rear);

%roll springs are in parallel, chassis in series 
A1 = (lambda^2 - (mu + 1)*lambda)/(lambda^2 - lambda - mu);
A2 = (dsf*m_sF)/(hG*m_T);
B1 = (mu*lambda)/(lambda^2 - lambda - mu);
B2 = (dsr * m_sR)/(hG * m_T);
C1 = (Zf * m_sF)/(hG *m_T);
D1 = (RL * m_usF)/(hG*m_T);
calc_rLLTD = A1*A2 - B1*B2 + C1+ D1;

Kch_record(i) = Kch;%record each sweep value 

rLLTD_record(i)= calc_rLLTD; 

end 

A1 = (lambda*dsf*(m_sR+m_sF)/(hG*m_T));
B1 = Zf*m_sF/(hG*m_T);
C1 = (RL*m_usF)/(hG*m_T);
calc_rLLTD = A1 + B1 + C1;
diff_LLTD = (rLLTD_record - calc_rLLTD)/(calc_rLLTD)*100;

figure
plot(Kch_record, rLLTD_record) 
xlabel('Chassis torsional stiffness(nm)')
ylabel('Resulting front LLTD')

figure
plot(Kch_record, diff_LLTD) 
xlabel('Chassis Torsional Stiffness(nm)')
ylabel('Error from infinitely stiff chassis')

 
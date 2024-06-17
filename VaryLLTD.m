clc, clearvars 

%% Suspension parameters 

trackwidth = 1.24; %m 

hG = .35; %CG height(m)

hsF = .35; %front CG height (m)

hsR = .35; %rear CG height(m)

m_T = 300; %Total vehicle mass(kg)

WD = .48; %Weight distribution relative to front

m_usF = 10; %Front unsprung mass(kg) (left and right combined)

m_usR = 10; %Rear unsprung mass(kg)

Kphi_t = 1200; %Total roll rate from springs (nm/deg)

Zf = .02795; %front roll center height(m)

Zr = .03556; %rear roll center height

RL =.205; %Tire loaded radius


%% Parameter calcs
dsf = hsF - Zf; %front roll moment arm (m)

dsr = hsR - Zr;

m_sF = (m_T*(WD)) - m_usF;

m_sR = (m_T * (1-WD)) - m_usR;

%% vary roll stiffness dist.
for (i = 1 : +1: 101)%sweep for LLTD(stiff chassis) in increments of .1
    Kch = 500;%Frame torsional stiffness(nm/deg)
    lambda = (i-1)*.01; %Front roll stiffness distribution
    mu = Kch/(Kphi_t);
    %roll springs are in parallel, chassis in series 
    A1 = (lambda^2 - (mu + 1)*lambda)/(lambda^2 - lambda - mu);
    A2 = (dsf*m_sF)/(hG*m_T);
    B1 = (mu*lambda)/(lambda^2 - lambda - mu);
    B2 = (dsr * m_sR)/(hG * m_T);
    C1 = (Zf * m_sF)/(hG *m_T);
    D1 = (RL * m_usF)/(hG*m_T);
    calc_rLLTD = A1*A2 - B1*B2 + C1+ D1;
    LLTD_ratio = calc_rLLTD/lambda;
    lambda_record(i) = lambda;%record each sweep value 
    rLLTD_record(i)= calc_rLLTD; 
    LLTD_ratio_record(i) = LLTD_ratio;
end

for (i = 1 : +1: 101)%sweep for LLTD(stiff chassis) in increments of .1
    Kch = 1200;%Frame torsional stiffness(nm/deg)
    lambda = (i-1)*.01; %Front roll stiffness distribution
    mu = Kch/(Kphi_t);
    %roll springs are in parallel, chassis in series 
    A1 = (lambda^2 - (mu + 1)*lambda)/(lambda^2 - lambda - mu);
    A2 = (dsf*m_sF)/(hG*m_T);
    B1 = (mu*lambda)/(lambda^2 - lambda - mu);
    B2 = (dsr * m_sR)/(hG * m_T);
    C1 = (Zf * m_sF)/(hG *m_T);
    D1 = (RL * m_usF)/(hG*m_T);
    calc_rLLTD = A1*A2 - B1*B2 + C1+ D1;
    LLTD_ratio = calc_rLLTD/lambda;
    lambda_record2(i) = lambda;%record each sweep value 
    rLLTD_record2(i)= calc_rLLTD; 
    LLTD_ratio_record2(i) = LLTD_ratio;
end

for (i = 1 : +1: 101)%Find load transfer for infinitly stiff chassis
    lambda = (i-1)*.01; %Front roll stiffness distribution
    %roll springs are in parallel, chassis in series 
    A1 = (lambda*dsf*(m_sR+m_sF)/(hG*m_T));
    B1 = Zf*m_sF/(hG*m_T);
    C1 = (RL*m_usF)/(hG*m_T);
    calc_rLLTD = A1 + B1 + C1;
    lambda_record3(i) = lambda;%record each sweep value 
    rLLTD_record3(i)= calc_rLLTD; 
end

plot(lambda_record, rLLTD_record) 
xlim([0, 1])
xlabel('Front Roll Stiffness Distribution')
ylabel('Front Load Transfer Distribution')

hold on 
plot(lambda_record2, rLLTD_record2) 
plot(lambda_record3, rLLTD_record3)
legend({'500nm/deg', '1200nm/deg', 'infinitely stiff'}, 'Location', 'southeast')

diff_LLTD = (rLLTD_record2 - rLLTD_record3)./(rLLTD_record3)*100;
figure
plot(lambda_record3,diff_LLTD)
xlabel('Front roll Stiffness Distribution')
ylabel('Error from infinitely stiff(%)')
legend('1200nm/deg')
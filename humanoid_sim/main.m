clc
close all
filename = 'data.csv';

T = readtable(filename); %check T.Properties
T = T(10:end,1:end-1);
VariableNames = T.Properties.VariableNames;
%%
Arr = table2array(T);
[m,n] = size(Arr);

%%
% plot q
for i=2:2:2
    figure(i)
    yy = i;
    plot(Arr(:,1), Arr(:,yy),'r','LineWidth',1.5);
    hold on
    plot(Arr(:,1), Arr(:,yy+1),'b','LineWidth',1.5);
    ylabel(cell2mat(VariableNames(yy)))
    xlabel('t')
    legend('target_q','q')
end

%%

for i=1:n
    figure(i)
    yy = i;
    plot(Arr(:,1), Arr(:,yy),'r','LineWidth',1.5);
    ylabel(cell2mat(VariableNames(yy)))
end

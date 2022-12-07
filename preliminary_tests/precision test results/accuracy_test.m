%% Set up the Import Options and import the data
opts = delimitedTextImportOptions("NumVariables", 4);

% Specify range and delimiter
opts.DataLines = [5, Inf];
opts.Delimiter = ":";

% Specify column names and types
opts.VariableNames = ["Var1", "x", "y", "mag"];
opts.SelectedVariableNames = ["x", "y", "mag"];
opts.VariableTypes = ["string", "double", "double", "double"];

% Specify file level properties
opts.ExtraColumnsRule = "ignore";
opts.EmptyLineRule = "read";

% Specify variable properties
opts = setvaropts(opts, "Var1", "WhitespaceRule", "preserve");
opts = setvaropts(opts, "Var1", "EmptyFieldRule", "auto");

% Import the data
accuracytest1 = readtable("C:\Users\mikip\Documents\P5_asphalt_robot\preliminary_tests\precision test results\accuracy_test1.txt", opts);
accuracytest2 = readtable("C:\Users\mikip\Documents\P5_asphalt_robot\preliminary_tests\precision test results\accuracy_test2.txt", opts);
accuracytest3 = readtable("C:\Users\mikip\Documents\P5_asphalt_robot\preliminary_tests\precision test results\accuracy_test3.txt", opts);
accuracytest4 = readtable("C:\Users\mikip\Documents\P5_asphalt_robot\preliminary_tests\precision test results\accuracy_test4.txt", opts);
accuracytest5 = readtable("C:\Users\mikip\Documents\P5_asphalt_robot\preliminary_tests\precision test results\accuracy_test5.txt", opts);


%% Extracting important data

alpha = norminv(1-0.00001);

% SET 1
average1 = sum(accuracytest1.mag)/length(accuracytest1.mag);
deviation1 = std(accuracytest1.mag);
significance1 = alpha*deviation1+average1;
highest_deviation1 = max(accuracytest1.mag);

disp(['E = ', num2str(average1),', sigma =',num2str(deviation1),', 95% Significance =',num2str(significance1),', Higest deviation =',num2str(highest_deviation1)])

% SET 2
average2 = sum(accuracytest2.mag)/length(accuracytest2.mag);
deviation2 = std(accuracytest2.mag);
significance2 = alpha*deviation2+average2;
highest_deviation2 = max(accuracytest2.mag);

disp(['E = ', num2str(average2),', sigma =',num2str(deviation2),', 95% Significance =',num2str(significance2),', Higest deviation =',num2str(highest_deviation2)])

% SET 3
average3 = sum(accuracytest3.mag)/length(accuracytest3.mag);
deviation3 = std(accuracytest3.mag);
significance3 = alpha*deviation3+average3;
highest_deviation3 = max(accuracytest3.mag);

disp(['E = ', num2str(average3),', sigma =',num2str(deviation3),', 95% Significance =',num2str(significance3),', Higest deviation =',num2str(highest_deviation3)])

% SET 4
average4 = sum(accuracytest4.mag)/length(accuracytest4.mag);
deviation4 = std(accuracytest4.mag);
significance4 = alpha*deviation4+average4;
highest_deviation4 = max(accuracytest4.mag);

disp(['E = ', num2str(average4),', sigma =',num2str(deviation4),', 95% Significance =',num2str(significance4),', Higest deviation =',num2str(highest_deviation4)])

% SET 5
average5 = sum(accuracytest5.mag)/length(accuracytest5.mag);
deviation5 = std(accuracytest5.mag);
significance5 = alpha*deviation5+average5;
highest_deviation5 = max(accuracytest5.mag);

disp(['E = ', num2str(average5),', sigma =',num2str(deviation5),', 95% Significance =',num2str(significance5),', Higest deviation =',num2str(highest_deviation5)])


%%
% ----------------------------------------------------------------------- %
% Project:         	Battery Parameter Modelling
% Created by:   	Anna Forster
% Unit:             EGH400 - Engineering Thesis
% Notes:            Initiating the environment to load the model 
% ----------------------------------------------------------------------- %

%% Clone Quadcopter Model from Mathworks

repoURL = 'https://github.com/annajforster/Quadcopter-Drone-Model-Simscape.git';
% branchName = 'R2022b';
destFolder = fullfile(pwd, 'Quadcopter-Drone-Model-Simscape');

if ~exist(destFolder, 'dir')
    fprintf('Cloning repository into:\n %s\n', destFolder);
    %fire-and-forget
    system([ 'git clone ' repoURL ' "' destFolder '"' ]);
else
    fprintf('Folder already exists, skipping clone:\n %s\n', destFolder);
end

%% Initiate Python Environment

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%Update this
pe = pyenv('Version', ... 
    'C:\Users\annaf\AppData\Local\Programs\Python\Python312\python.exe');
disp(pe)

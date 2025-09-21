function push_new_branch(branchName, commitMsg, repoPath)
% Create a new git branch in a MATLAB project, commit changes, and push to GitHub.
% Usage:
%   push_new_branch('feature/my-change', 'Add X and fix Y')
%   push_new_branch('bugfix/issue-123', 'Fix Z', 'C:\path\to\repo')

    arguments
        branchName (1,1) string
        commitMsg  (1,1) string
        repoPath   (1,1) string = string(pwd)
    end

    % Move to repo
    oldpwd = pwd;
    cleanup = onCleanup(@() cd(oldpwd));
    cd(repoPath);

    % 0) Sanity checks
    mustHaveGit();
    mustBeRepo();

    % 1) Ensure we’re at the repo root (useful if you’re in a subfolder)
    repoRoot = runGit("rev-parse --show-toplevel");
    cd(strtrim(repoRoot));

    % 2) Check/print current remote
    remotes = strtrim(runGit("remote -v"));
    if remotes == ""
        error(['No git remote configured. Add one first, e.g.:' newline ...
               '  git remote add origin https://github.com/<user>/<repo>.git']);
    else
        fprintf('Remote(s):\n%s\n', remotes);
    end

    % 3) Create the new branch (from current HEAD)
    % Use `checkout -b` for maximum compatibility.
    existingBranches = runGit("branch --list " + q(branchName));
    if strlength(strtrim(existingBranches)) > 0
        fprintf('Branch "%s" already exists locally. Switching to it...\n', branchName);
        runGit("checkout " + q(branchName));
    else
        fprintf('Creating branch "%s"...\n', branchName);
        runGit("checkout -b " + q(branchName));
    end

    % 4) Stage & commit (includes new/untracked files)
    runGit("add -A");

    % If nothing to commit, `git commit` returns nonzero; handle gracefully
    [status, out] = system(buildGitCmd("commit -m " + q(commitMsg)));
    if status ~= 0
        if contains(out, "nothing to commit", 'IgnoreCase', true)
            fprintf('Nothing to commit (working tree clean). Proceeding to push.\n');
        else
            error("git commit failed:\n%s", out);
        end
    else
        fprintf("Committed: %s\n", commitMsg);
    end

    % 5) Push with upstream tracking
    fprintf('Pushing branch "%s" to origin and setting upstream...\n', branchName);
    runGit("push -u origin " + q(branchName));

    % 6) Show the branch + last commit for confirmation
    fprintf('\nDone ✅\n');
    fprintf('Current branch: %s\n', strtrim(runGit("rev-parse --abbrev-ref HEAD")));
    fprintf('Last commit:\n%s\n', runGit("log -1 --oneline --decorate"));

    % --- helpers ---
    function mustHaveGit()
        [s, ~] = system('git --version');
        if s ~= 0
            error('Git is not available on PATH. Install Git and/or add it to your PATH.');
        end
    end

    function mustBeRepo()
        [s, ~] = system('git rev-parse --is-inside-work-tree');
        if s ~= 0
            error('This folder is not inside a git repository: %s', repoPath);
        end
    end

    function out = runGit(args)
        [s, out] = system(buildGitCmd(args));
        if s ~= 0
            error("git %s failed:\n%s", args, out);
        end
    end

    function cmd = buildGitCmd(args)
        % Build a platform-safe git command
        if ispc
            cmd = "git " + args;
        else
            cmd = "git " + args;
        end
    end

    function s = q(strIn)
        % Quote for shell safety
        s = '"' + strrep(string(strIn), '"', '\"') + '"';
    end
end

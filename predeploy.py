import argparse
import logging
import os
import os.path
import subprocess
import sys

logger = logging.getLogger('predeploy')

def parse_args():
  """
  Parse command-line args to extract the robot path, as determined by pyfrc's
  cli_deploy.py (which calls the pre-deploy script).
  """
  parser = argparse.ArgumentParser()
  parser.add_argument("--robot_path", help="path to robot.py, as determined by cli_deploy.py")
  return parser.parse_args(sys.argv[1:])

def command_output(list_of_args, one_line=False, as_str=True):
  """
  Return the output of the given command, as bytes or as a string.
  Params:
    list_of_args (list[str]): command and arguments to execute
    one_line (bool): if True, strip newline from end of output
    as_str (bool): if True, decode bytes from terminal to str before returning
  """
  result = subprocess.check_output(list_of_args)
  if one_line: result = result.strip()
  if as_str: result = result.decode('utf-8')
  return result

def git_revision():
  """
  Return the Git revision (commit hash) of the current directory.

  Assumes that the current directory is a Git repo.
  """
  cmdlist = ['git', 'rev-parse', '--short', 'HEAD']
  return command_output(cmdlist, one_line=True)

def git_branch():
  """
  Return the Git working branch of the current directory.

  Assumes that the current directory is a Git repo.
  """
  cmdlist = ['git', 'rev-parse', '--abbrev-ref', 'HEAD']
  return command_output(cmdlist, one_line=True)

def git_tags():
  """
  Return all git tags that point to the current version.

  WARNING: If your repo has no tags, this will not return anything useful,
  due to the behavior of "git describe --tags --exact-match" in that case.

  Assumes that the current directory is a Git repo.
  """
  cmdlist = ["git", "describe", "--tags", "--exact-match"]
  output = command_output(cmdlist).strip().split()
  return [str(tag) for tag in output]

def git_modified_files():
  """
  Return a list of all files not commited to Git in the current directory.

  Assumes that the current directory is a Git repo.
  """
  cmdlist = ['git', 'ls-files', '--modified']
  output = command_output(cmdlist, one_line=True)
  return [str(filename) for filename in output.split('\n')]

def write_data_to_file(dirname, filename, data):
  """
  Simple function to write data to {dirname/filename}
  """
  with open(os.path.join(dirname, filename), "w") as f:
    f.write(data)
    f.write('\n')

def write_files(args):
  """
  Write Git information to various git-* files in the robot_path directory
  found in args, and log the information.
  """
  targetdir = args.robot_path
  git_rev = git_revision()
  logger.info(f'Git revision: {git_rev}')
  write_data_to_file(targetdir, 'git-revision.txt', git_rev)
  git_br = git_branch()
  logger.info(f'Git branch: {git_br}')
  write_data_to_file(targetdir, 'git-branch.txt', git_br)
  git_modified = git_modified_files()
  if git_modified:
    logger.warning(f'Modified files not committed to Git: {git_modified}')
  else:
    logger.info(f'All files committed to Git')
  write_data_to_file(targetdir, 'git-modified-files.txt',
                     '\n'.join(git_modified))
  logger.info(f'Updated git-* files in {args.robot_path}')


if __name__ == '__main__':
  args = parse_args()
  write_files(args)

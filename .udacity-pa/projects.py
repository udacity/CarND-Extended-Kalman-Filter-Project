import argparse
from udacity_pa import udacity

nanodegree = 'nd013'
projects = ['extended-kalman-filter']

def submit(args):
  parser = argparse.ArgumentParser(description='submits the project.')
  parser.add_argument('filenames', nargs='*', help='files to include (those included as starter code by default')

  local_args = parser.parse_args(args.args)

  default_filenames = [
    'src/FusionEKF.cpp',
    'src/FusionEKF.h',
    'src/kalman_filter.cpp',
    'src/kalman_filter.h',
    'src/tools.cpp',
    'src/tools.h'
  ]

  if len(local_args.filenames) == 0:
    filenames = default_filenames
  else:
    filenames = local_args.filenames

  udacity.submit(nanodegree, projects[0], filenames,
                 environment = args.environment,
                 jwt_path = args.jwt_path)

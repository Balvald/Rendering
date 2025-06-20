project('Raytracer', ['c', 'cpp'], default_options : ['cpp_std=c++20'], )

subdir('models')

if host_machine.system() == 'windows'
    # We are on Windows
    # Add MSMPI dependency by hand.
    # add_global_arguments('-IC:/Program Files (x86)/Microsoft SDKs/MPI/Include/', language : 'cpp')
    # add_global_link_arguments('-LC:/Program Files (x86)/Microsoft SDKs/MPI/Lib/x64/', language : 'cpp')
    # add_global_link_arguments('msmpi.lib', language : 'cpp')
    # mpi = dependency('mpi', language: 'cpp', required: false)
    omp = dependency('openmp')
elif host_machine.system() == 'linux'
    # We are on Linux
    # mpi = dependency('mpi', language: 'cpp')
    omp = dependency('openmp')
elif host_machine.system() == 'darwin'
    # We are on Mac (fp: Don't own a Mac, so this is untested)
    # mpi = dependency('mpi', language: 'cpp')
    omp = dependency('openmp')
    message('Darwin Platform. Hope you have MPI available as its needed.')
    message('You\'re on your own if this doesn\'t work!')
else
    # We are on some other unknown platform
    # mpi = dependency('mpi', language: 'cpp')
    omp = dependency('openmp')
    message('Unknown platform. Hope you have MPI available as its needed.')
    message('You\'re on your own if this doesn\'t work!')
endif

eigen = dependency('eigen3', version: '>=3.4.0')

lib_sources = [ # All source files (excluding headers)
]

lib = static_library(
    'lib',  # Unique name
    lib_sources,
    dependencies : [eigen, # mpi,
                    omp]
)

lib_incdirs = include_directories('.')

executable('raytracer', 'main.cpp',
    include_directories : [lib_incdirs],
	# link_with : [lib],
    dependencies : [eigen, # mpi,
                    omp])
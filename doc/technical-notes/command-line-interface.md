# Command-Line Interface

This note describes the shared command-line helper implemented by
`pe::CommandLineInterface`.

## Source Files

- `pe/support/CommandLineInterface.h`: class declaration, singleton access,
  accessors, and default template hooks.
- `src/support/CommandLineInterface.cpp`: option registration, parsing,
  evaluation, and solver-specific template specializations.

The class wraps Boost.Program_options for PE examples and applications that use
the shared support layer. It is not a complete description of every option an
individual executable may add on top.

## Lifecycle

The interface is a singleton accessed through `CommandLineInterface::getInstance()`.
It stores:

- `desc_`: the Boost option description used while parsing.
- `vm_`: the parsed Boost variable map.

Typical use is:

1. Obtain the singleton with `CommandLineInterface::getInstance()`.
2. Optionally add executable-specific options to `getDescription()`.
3. Call `parse(argc, argv)`.
4. Call `evaluateOptions()` after the collision system and solver state are
   available.

`parse()` handles `--help`/`-h` directly by printing the registered options and
exiting. It also catches Boost.Program_options errors, prints the error and the
option list, and exits.

## Built-In Options

The constructor registers the options that are common to the shared support
layer.

| Option | Availability | Effect |
|--------|--------------|--------|
| `--help`, `-h` | Always | Print option help and exit during parsing. |
| `--seed`, `-s` | Always | Set the random seed during evaluation. The MPI rank is added to the provided value. |
| `--no-povray` | Always | Register a common switch for disabling POV-Ray output. Consumers must check the parsed variable map where needed. |
| `--no-vtk` | Always | Register a common switch for disabling VTK output. Consumers must check the parsed variable map where needed. |
| `--no-irrlicht` | `HAVE_IRRLICHT` | Register a common switch for disabling Irrlicht visualization. Consumers must check the parsed variable map where needed. |

The visualization switches are registered here, but this class does not by
itself disable renderer instances. Executables or visualization setup code need
to consult `getVariablesMap()` and act on the selected flags.

## Contact Solver Options

The constructor calls `addContactSolverOptions<ContactSolver>()`, where
`ContactSolver` is the configured solver type from the collision system setup.
The default template in the header is intentionally a no-op. Concrete
specializations in the implementation file add options for supported solver
families.

For `response::ContactSolver<Config>` the CLI registers and evaluates:

| Option | Effect |
|--------|--------|
| `--max-iterations` | Set the maximum number of solver iterations per time step. |
| `--threshold`, `-t` | Set the convergence threshold. |

For OpenCL builds, `response::OpenCLSolver<Config, NullType, NullType>` first
inherits the generic contact solver options and then adds:

| Option | Effect |
|--------|--------|
| `--colors`, `-c` | Set the color limit used during the solving process. |
| `--omega`, `-w` | Set the relaxation parameter. |

`evaluateOptions()` applies solver options through
`theCollisionSystem()->getContactSolver()`. This means solver-specific CLI
support depends on the compile-time solver selected in the PE configuration.

## OpenCL Options

When PE is built with `HAVE_OPENCL`, the constructor also registers:

| Option | Effect |
|--------|--------|
| `--list-platforms` | Print available OpenCL platforms. |
| `--list-devices <platform>` | Print available devices for a platform index. |
| `--platform <index>` | Select the OpenCL platform. |
| `--device <index>` | Select the OpenCL device. |

`evaluateOptions()` performs the OpenCL queries or selection and then constructs
the OpenCL system interface, printing the selected platform and device.

## Extension Points

To add shared CLI support for another contact solver type:

1. Add a specialization of `addContactSolverOptions<T>()` in
   `src/support/CommandLineInterface.cpp`.
2. Add the matching specialization of `evaluateContactSolverOptions<T>()`.
3. Keep option names stable and avoid solver-specific options that conflict with
   executable-specific options.
4. Confirm the configured `ContactSolver` typedef reaches the specialization
   through the active collision system configuration.

The no-op template definitions in the header are intentional fallback behavior:
solver types without shared CLI options still compile and simply do not add or
evaluate solver-specific flags.

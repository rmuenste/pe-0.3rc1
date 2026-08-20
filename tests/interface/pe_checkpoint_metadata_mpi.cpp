// Two-rank checkpoint write/read test.
//
// The serial suite cannot reach the parts of the checkpoint path that only exist under MPI, and
// those are exactly the parts where a mistake is silent:
//
//   1. The scratch filename must be RANK-INVARIANT. MPI_File_open is collective and requires an
//      identical filename on every rank, so a scratch name carrying each rank's own pid makes the
//      write erroneous. That is not a hypothetical: it is what the first version of this feature
//      did, and a serial-only suite passes it happily.
//   2. The sidecar's bodyCount describes the whole file, so the per-rank marshalled counts must be
//      summed. Rank 0's own count is not the answer.
//
// No domain decomposition is set up, deliberately. defineLocalDomain() throws
// "Selected configuration is not MPI parallel" for every configuration in this tree --
// ParallelTrait has no specialisation that sets value = 1, so the specialisation block in
// pe/core/ParallelTrait.h is empty and the primary template's 0 always wins. That is pre-existing
// and out of scope here. It does not matter for this test: the checkpoint write is collective
// over MPI_COMM_WORLD and each rank simply contributes its own local bodies, which is exactly the
// shape the two properties above need.

#include <pe/core.h>
#include <pe/core/Materials.h>
#include <pe/core/TimeStep.h>
#include <pe/util/CheckpointMetadata.h>
#include <pe/util/Checkpointer.h>

#include <boost/filesystem.hpp>

#include <cstdlib>
#include <iostream>
#include <string>

#include <mpi.h>

namespace {

namespace fs = boost::filesystem;

int failures = 0;
int rank = 0;

// Only rank 0 reports, so a failure is not multiplied by the rank count.
void check(bool condition, const std::string& message) {
  if (!condition) {
    ++failures;
    if (rank == 0) {
      std::cerr << "FAIL: " << message << std::endl;
    }
  }
}

const unsigned spheresPerRank = 3;

} // namespace

int main(int argc, char** argv) {
  MPI_Init(&argc, &argv);

  int size = 0;
  MPI_Comm_rank(MPI_COMM_WORLD, &rank);
  MPI_Comm_size(MPI_COMM_WORLD, &size);
  if (size != 2) {
    if (rank == 0) {
      std::cerr << "pe_checkpoint_metadata_mpi requires exactly two MPI ranks" << std::endl;
    }
    MPI_Abort(MPI_COMM_WORLD, 2);
  }

  try {
    pe::WorldID world = pe::theWorld();
    world->setGravity(0.0, 0.0, 0.0);

    pe::TimeStep::stepsize(pe::real(0.004));
    pe::setCheckpointIdentity(pe::real(4.8), 1200u, "mpi pairing tag");

    const size_t builtins = pe::Material::count();
    const pe::MaterialID mat = pe::createMaterial(
        "mpi-particle", pe::real(1.1), pe::real(0.1), pe::real(0.05), pe::real(0.05),
        pe::real(0.3), pe::real(300), pe::real(1e6), pe::real(1e5), pe::real(2e5));

    // Each rank seeds its own bodies, so both ranks contribute a non-empty chunk to the file --
    // which is the precondition for the body-count reduction to be observable at all.
    const pe::real sign = (rank == 0) ? pe::real(-1.0) : pe::real(1.0);
    for (unsigned i = 0; i < spheresPerRank; ++i) {
      const pe::Vec3 position(sign * (pe::real(0.5) + pe::real(i)), 0.0, 0.0);
      pe::createSphere(static_cast<pe::id_t>(rank * 100 + i), position, pe::real(0.1), mat);
    }
    check(world->size() == spheresPerRank, "each rank should hold its own seeds");

    // Both ranks must agree on the directory, so rank 0's value is broadcast below; the local
    // pid here is therefore only rank 0's, and every rank ends up with that one.
    const fs::path checkpoints =
        fs::temp_directory_path() / ("pe-checkpoint-mpi-" + std::to_string(::getpid()));
    std::string dir = checkpoints.string();
    unsigned long dirLength = dir.size();
    MPI_Bcast(&dirLength, 1, MPI_UNSIGNED_LONG, 0, MPI_COMM_WORLD);
    dir.resize(dirLength);
    MPI_Bcast(&dir[0], static_cast<int>(dirLength), MPI_CHAR, 0, MPI_COMM_WORLD);
    const fs::path checkpointDir(dir);

    // THE test: a collective write whose scratch name must be identical on both ranks.
    pe::writeCheckpoint(checkpointDir, "mpi_seed");
    MPI_Barrier(MPI_COMM_WORLD);

    if (rank == 0) {
      const fs::path peb = checkpointDir / "mpi_seed.peb";
      const fs::path sidecar = pe::checkpointMetadataPath(checkpointDir, "mpi_seed");
      check(fs::exists(peb), "MPI write did not produce a .peb");
      check(fs::exists(sidecar), "MPI write did not produce a sidecar");

      // A rank-dependent scratch name shows up as leftovers, so enumerate rather than trust.
      // Both markers are checked: `.tmp-` for the in-progress copies, `.prev-` for the preserved
      // previous sidecar, which must be gone once the new one is published.
      size_t scratchFiles = 0;
      for (fs::directory_iterator it(checkpointDir), end; it != end; ++it) {
        const std::string name = it->path().string();
        if (name.find(".tmp-") != std::string::npos ||
            name.find(".prev-") != std::string::npos) {
          ++scratchFiles;
          std::cerr << "  leftover scratch file: " << name << std::endl;
        }
      }
      check(scratchFiles == 0, "MPI write left scratch files behind");

      const pe::CheckpointMetadata metadata = pe::readCheckpointMetadata(sidecar);
      check(metadata.present, "MPI sidecar should be present");
      check(metadata.pebBytes == static_cast<uint64_t>(fs::file_size(peb)),
            "MPI sidecar records the wrong .peb size");
      check(metadata.timeStep == 1200u, "MPI sidecar lost the driver step");
      check(metadata.pairingTag == "mpi pairing tag", "MPI sidecar lost the pairing tag");
      check(metadata.materials.size() == builtins + 1,
            "MPI sidecar did not record the full material table");

      // Finding 2: the count must span both ranks, not just rank 0's chunk.
      check(metadata.bodyCount == 2 * spheresPerRank,
            "MPI sidecar bodyCount is " + std::to_string(metadata.bodyCount) + ", expected " +
                std::to_string(2 * spheresPerRank) +
                " (per-rank count leaked instead of the total)");
    }

    MPI_Barrier(MPI_COMM_WORLD);

    // Reading back is collective too, and must reconstruct every rank's chunk.
    const pe::CheckpointMetadata readBack = pe::readCheckpoint(checkpointDir, "mpi_seed");
    check(readBack.present, "MPI read did not see the sidecar");
    check(readBack.bodyCount == 2 * spheresPerRank, "MPI read saw the wrong bodyCount");

    MPI_Barrier(MPI_COMM_WORLD);
    if (rank == 0) {
      fs::remove_all(checkpointDir);
    }

    int localFailures = failures;
    int totalFailures = 0;
    MPI_Allreduce(&localFailures, &totalFailures, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);

    if (rank == 0) {
      if (totalFailures != 0) {
        std::cerr << "pe_checkpoint_metadata_mpi: " << totalFailures << " check(s) failed"
                  << std::endl;
      } else {
        std::cout << "pe_checkpoint_metadata_mpi: PASS" << std::endl;
      }
    }

    MPI_Finalize();
    return totalFailures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;
  } catch (const std::exception& e) {
    std::cerr << "pe_checkpoint_metadata_mpi: rank " << rank << ": unexpected exception: "
              << e.what() << std::endl;
    MPI_Abort(MPI_COMM_WORLD, 1);
    return EXIT_FAILURE;
  }
}

// Collective failure contracts: each scenario is a fresh two-rank process. CTest's timeout
// turns a missing agreement point into a failure instead of an indefinitely blocked runner.
#include <pe/core.h>
#include <pe/core/BodyBinaryWriter.h>
#include <pe/util/Checkpointer.h>
#include <pe/util/CheckpointMetadata.h>
#include <boost/filesystem.hpp>
#include <mpi.h>
#include <unistd.h>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>

// Interpose MPI's public I/O entry points using the standard PMPI interface. Inject returned
// errors on rank 1 after a successful underlying operation so error paths are deterministic,
// including completion/close errors that are difficult to reproduce with filesystem fixtures.
namespace {
std::string fault;
bool armed = false;
int faultRank = 0;
bool inject(const char* operation) { return armed && faultRank == 1 && fault == operation; }
}
extern "C" int MPI_File_iwrite_at(MPI_File file, MPI_Offset offset, const void* buffer,
                                 int count, MPI_Datatype type, MPI_Request* request)
{
   if(inject("submit")) { *request = MPI_REQUEST_NULL; return MPI_ERR_IO; }
   return PMPI_File_iwrite_at(file, offset, buffer, count, type, request);
}
extern "C" int MPI_Wait(MPI_Request* request, MPI_Status* status)
{
   const int result = PMPI_Wait(request, status);
   return result == MPI_SUCCESS && inject("complete") ? MPI_ERR_IO : result;
}
extern "C" int MPI_File_close(MPI_File* file)
{
   const int result = PMPI_File_close(file);
   return result == MPI_SUCCESS && inject("close") ? MPI_ERR_IO : result;
}
extern "C" int MPI_File_read_at(MPI_File file, MPI_Offset offset, void* buffer,
                               int count, MPI_Datatype type, MPI_Status* status)
{
   const int result = PMPI_File_read_at(file, offset, buffer, count, type, status);
   return result == MPI_SUCCESS && inject("read") ? MPI_ERR_IO : result;
}

namespace fs = boost::filesystem;

void require(bool condition, const std::string& message)
{
   if(!condition) throw std::runtime_error(message);
}

uint32_t readOffset(std::istream& in)
{
   unsigned char bytes[4];
   in.read(reinterpret_cast<char*>(bytes), 4);
   require(bool(in), "Cannot read checkpoint offset");
   return (uint32_t(bytes[0]) << 24) | (uint32_t(bytes[1]) << 16) |
          (uint32_t(bytes[2]) << 8) | bytes[3];
}

int main(int argc, char** argv)
{
   MPI_Init(&argc, &argv);
   int rank = 0, size = 0;
   MPI_Comm_rank(MPI_COMM_WORLD, &rank);
   MPI_Comm_size(MPI_COMM_WORLD, &size);
   try {
      require(size == 2 && argc == 2, "Requires two ranks and a scenario");
      const std::string scenario(argv[1]);
      fault = scenario;
      faultRank = rank;
      long token = rank == 0 ? static_cast<long>(::getpid()) : 0;
      MPI_Bcast(&token, 1, MPI_LONG, 0, MPI_COMM_WORLD);
      const fs::path root = fs::temp_directory_path() /
         ("pe-checkpoint-failure-" + scenario + "-" + std::to_string(token));
      const fs::path dir = root / "checkpoints";
      const fs::path peb = dir / "seed.peb";
      const fs::path sidecar = dir / "seed.peinfo";
      const fs::path pebTemp = pe::checkpointTempPath(peb, token);
      const fs::path sidecarTemp = pe::checkpointTempPath(sidecar, token);
      if(rank == 0) fs::create_directories(root);
      MPI_Barrier(MPI_COMM_WORLD);

      const pe::MaterialID material = pe::createMaterial(
         "failure-test", 1.1, 0.1, 0.05, 0.05, 0.3, 300, 1e6, 1e5, 2e5);
      pe::createSphere(rank, pe::Vec3(rank ? 1 : -1, 0, 0), 0.1, material);
      pe::setCheckpointIdentity(1.0, 10, "previous pair");

      // All scenarios except directory preflight start from a valid checkpoint. This also
      // makes recovery of the old sidecar after pre-publication failure observable.
      if(scenario != "directory" && scenario != "binary-open") pe::writeCheckpoint(dir, "seed");
      if(rank == 0) {
         if(scenario == "directory") std::ofstream((root / "blocker").string()) << "file";
         else if(scenario == "open") fs::create_directory(pebTemp);
         else if(scenario == "binary-open") fs::create_directories(peb);
         else if(scenario == "rename") {
            fs::rename(peb, root / "previous.peb");
            fs::create_directory(peb);
         }
         else if(scenario == "sidecar") fs::create_directory(sidecarTemp);
         else if(scenario == "preserve") {
            const fs::path marker = dir / ("seed.peinfo.prev-" + std::to_string(token));
            fs::create_directory(marker);
            std::ofstream((marker / "blocker").string()) << "file";
         }
         else if(scenario == "metadata" || scenario == "missing") {
            fs::create_directory(root / "rank1");
            fs::copy_file(sidecar, root / "rank1" / "seed.peinfo");
            if(scenario == "metadata") {
               fs::copy_file(peb, root / "rank1" / "seed.peb");
               std::ofstream((root / "rank1" / "seed.peinfo").string()) << "metadataVersion broken\n";
            }
         }
         else if(scenario == "body") {
            // Poison only rank 1's first geometry tag; the lengths and metadata remain valid.
            // Prefix has 13 bytes; offsets are global, rank 0, rank 1, end, in network order.
            std::fstream file(peb.string(), std::ios::in | std::ios::out | std::ios::binary);
            file.seekg(13 + 2 * 4);
            const uint32_t rank1Offset = readOffset(file);
            file.seekp(rank1Offset + sizeof(pe::id_t));
            const char invalid = static_cast<char>(255);
            file.write(&invalid, 1);
            require(bool(file), "Cannot corrupt body tag");
         }
      }
      MPI_Barrier(MPI_COMM_WORLD);
      pe::setCheckpointIdentity(2.0, 20, "new pair");

      // A material mismatch can be rank-local even when both ranks see the same sidecar.
      if(scenario == "material" && rank == 1) {
         pe::CheckpointMetadata metadata = pe::readCheckpointMetadata(sidecar);
         metadata.materials.back().density = 2.0;
         const fs::path localDir = root / "rank1";
         fs::create_directories(localDir);
         fs::copy_file(peb, localDir / "seed.peb");
         pe::writeCheckpointMetadata(localDir / "seed.peinfo", metadata);
      }
      MPI_Barrier(MPI_COMM_WORLD);

      const pe::CheckpointerID checkpointer = pe::activateCheckpointer(dir, 100, 0, 100);
      pe::BodyBinaryWriter binaryWriter;
      std::string message;
      armed = true;
      try {
         if(scenario == "directory")
            pe::writeCheckpoint(rank == 1 ? root / "blocker" / "checkpoints" : dir, "seed");
         else if(scenario == "metadata" || scenario == "missing" || scenario == "material")
            pe::readCheckpoint(rank == 1 ? root / "rank1" : dir, "seed");
         else if(scenario == "body" || scenario == "read") checkpointer->read("seed");
         else if(scenario == "binary-open") binaryWriter.writeFile(peb.string().c_str());
         else checkpointer->write("seed");
      }
      catch(const std::runtime_error& error) { message = error.what(); }
      armed = false;

      int localCaught = !message.empty(), caught = 0;
      MPI_Allreduce(&localCaught, &caught, 1, MPI_INT, MPI_SUM, MPI_COMM_WORLD);
      require(caught == 2, "Both ranks must throw");
      std::string ownerMessage = message;
      int length = static_cast<int>(ownerMessage.size());
      MPI_Bcast(&length, 1, MPI_INT, 0, MPI_COMM_WORLD);
      ownerMessage.resize(length);
      MPI_Bcast(&ownerMessage[0], length, MPI_CHAR, 0, MPI_COMM_WORLD);
      require(message == ownerMessage, "Ranks must report the same diagnostic");
      const int expectedRank = scenario == "directory" || scenario == "metadata" ||
         scenario == "missing" || scenario == "material" || scenario == "body" || scenario == "read" ||
         scenario == "submit" || scenario == "complete" || scenario == "close" ? 1 : 0;
      require(message.find("rank " + std::to_string(expectedRank)) != std::string::npos,
              "Diagnostic must name the failing rank: " + message);
      require(message.find(root.string()) != std::string::npos,
              "Diagnostic must name the checkpoint path: " + message);

      if(rank == 0) {
         std::cout << scenario << ": " << message << std::endl;
         if(scenario == "sidecar") {
            require(fs::exists(peb) && !fs::exists(sidecar),
                    "Published bodies must never be paired with the previous sidecar");
         }
         else if(scenario == "open" || scenario == "rename" || scenario == "preserve" ||
                 scenario == "submit" || scenario == "complete" || scenario == "close") {
            require(pe::readCheckpointMetadata(sidecar).pairingTag == "previous pair",
                    "Failure before publication must preserve the old sidecar");
         }
         // Remove test-injected obstacles before checking for artifacts generated by PE.
         if(scenario == "preserve")
            fs::remove_all(dir / ("seed.peinfo.prev-" + std::to_string(token)));
         if(fs::exists(dir)) for(fs::directory_iterator it(dir), end; it != end; ++it) {
            const std::string name = it->path().filename().string();
            require(name.find(".tmp-") == std::string::npos && name.find(".prev-") == std::string::npos,
                    "Leftover checkpoint scratch file: " + name);
         }
      }
      MPI_Barrier(MPI_COMM_WORLD);
      // A failed operation must leave the writer/reader usable for a subsequent round trip.
      if(rank == 0 && (scenario == "rename" || scenario == "binary-open")) fs::remove_all(peb);
      MPI_Barrier(MPI_COMM_WORLD);
      if(scenario == "binary-open") binaryWriter.writeFile(peb.string().c_str());
      checkpointer->write("recovered");
      require(checkpointer->read("recovered").pairingTag == "new pair", "Recovery failed");
      MPI_Barrier(MPI_COMM_WORLD);
      if(rank == 0) fs::remove_all(root);
      MPI_Finalize();
      return 0;
   }
   catch(const std::exception& error) {
      std::cerr << "rank " << rank << ": " << error.what() << std::endl;
      MPI_Abort(MPI_COMM_WORLD, 1);
      return 1;
   }
}

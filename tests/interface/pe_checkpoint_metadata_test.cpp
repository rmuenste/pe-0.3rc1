// Contract tests for checkpoint identity metadata.
//
// Covers the five properties the checkpoint sidecar exists to provide:
//   1. roundtrip        -- what is written is what is read back, field for field
//   2. legacy           -- a `.peb` without a sidecar still loads, reported as "not present"
//   3. truncation       -- a short `.peb` or a short sidecar is refused, never half-read
//   4. materials        -- the recorded material table is reinstated at identical indices
//   5. mismatch guard   -- validateResumeExpectation throws iff an expectation is set and violated
//
// Cases 1-3 and 5 are pure file/struct level and share one process. Case 4 needs a world with
// bodies and therefore mutates the process-global body storage; it runs last for that reason.

#include <pe/core.h>
#include <pe/core/Materials.h>
#include <pe/core/TimeStep.h>
#include <pe/util/CheckpointMetadata.h>
#include <pe/util/Checkpointer.h>

#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

// A single-rank test still has to initialise MPI when the library was built with it: the
// checkpoint write path is collective (MPI_Exscan in BodyBinaryWriter, the scratch-token
// broadcast, the body-count reduction) and calling any of those before MPI_Init aborts the
// process, even at size 1.
#if HAVE_MPI
#include <mpi.h>
#endif

#include <cstdlib>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

namespace fs = boost::filesystem;

int failures = 0;

void check(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << std::endl;
    ++failures;
  }
}

fs::path workDir() {
  static const fs::path dir =
      fs::temp_directory_path() / fs::unique_path("pe-checkpoint-metadata-%%%%%%%%");
  return dir;
}

pe::CheckpointMetadata sampleMetadata() {
  pe::CheckpointMetadata metadata;
  metadata.present = true;
  metadata.version = pe::checkpointMetadataVersion;
  metadata.timeSource = pe::checkpointTimeFromDriver;
  metadata.simulationTime = pe::real(12.0);
  metadata.timeStep = 2400u;
  metadata.stepSize = pe::real(0.005);
  metadata.bodyCount = 7u;
  metadata.pebBytes = 4096u;
  metadata.pairingTag = "ff dump 3 rank 0";  // Spaces on purpose: the tag is free-form.

  pe::CheckpointMaterialRecord record;
  record.name = "particle material";
  record.density = pe::real(1.13);
  record.cor = pe::real(0.1);
  record.csf = pe::real(0.05);
  record.cdf = pe::real(0.05);
  record.poisson = pe::real(0.3);
  record.young = pe::real(300);
  record.stiffness = pe::real(1e6);
  record.dampingN = pe::real(1e5);
  record.dampingT = pe::real(2e5);
  metadata.materials.push_back(record);

  return metadata;
}

//=================================================================================================
// 1. Roundtrip
//=================================================================================================
void testRoundtrip() {
  const fs::path file = workDir() / "roundtrip.peinfo";
  const pe::CheckpointMetadata written = sampleMetadata();
  pe::writeCheckpointMetadata(file, written);

  const pe::CheckpointMetadata read = pe::readCheckpointMetadata(file);

  check(read.present, "roundtrip: metadata should be present");
  check(read.version == written.version, "roundtrip: version");
  check(read.timeSource == written.timeSource, "roundtrip: timeSource");
  check(read.simulationTime == written.simulationTime, "roundtrip: simulationTime is not exact");
  check(read.timeStep == written.timeStep, "roundtrip: timeStep");
  check(read.stepSize == written.stepSize, "roundtrip: stepSize is not exact");
  check(read.bodyCount == written.bodyCount, "roundtrip: bodyCount");
  check(read.pebBytes == written.pebBytes, "roundtrip: pebBytes");
  check(read.pairingTag == written.pairingTag,
        "roundtrip: pairingTag lost its spaces or content: '" + read.pairingTag + "'");
  check(read.materials.size() == 1, "roundtrip: material count");
  if (read.materials.size() == 1) {
    const pe::CheckpointMaterialRecord& m = read.materials[0];
    const pe::CheckpointMaterialRecord& w = written.materials[0];
    check(m.name == w.name, "roundtrip: material name with spaces: '" + m.name + "'");
    check(m.density == w.density, "roundtrip: material density is not exact");
    check(m.cor == w.cor && m.csf == w.csf && m.cdf == w.cdf, "roundtrip: material friction terms");
    check(m.poisson == w.poisson && m.young == w.young && m.stiffness == w.stiffness,
          "roundtrip: material elastic terms");
    check(m.dampingN == w.dampingN && m.dampingT == w.dampingT, "roundtrip: material damping");
  }

  // An empty tag is a legitimate value, not a missing directive.
  pe::CheckpointMetadata untagged = sampleMetadata();
  untagged.pairingTag.clear();
  const fs::path untaggedFile = workDir() / "untagged.peinfo";
  pe::writeCheckpointMetadata(untaggedFile, untagged);
  check(pe::readCheckpointMetadata(untaggedFile).pairingTag.empty(),
        "roundtrip: empty pairing tag should read back as empty");
}

//=================================================================================================
// 2. Legacy acceptance
//=================================================================================================
void testLegacyAcceptance() {
  const fs::path missing = workDir() / "no-such-checkpoint.peinfo";
  const pe::CheckpointMetadata metadata = pe::readCheckpointMetadata(missing);
  check(!metadata.present, "legacy: an absent sidecar must be reported, not thrown on");
  check(metadata.materials.empty(), "legacy: absent sidecar must carry no material table");

  // A legacy checkpoint with no expectation set is accepted; the loud warning is emitted by
  // readCheckpoint(), which is exercised by the ATC resume roundtrip case.
  pe::ResumeExpectation none;
  bool accepted = true;
  try {
    pe::validateResumeExpectation(metadata, none, "legacy.peb");
  } catch (const std::exception&) {
    accepted = false;
  }
  check(accepted, "legacy: an empty expectation must accept a legacy checkpoint");
}

//=================================================================================================
// 3. Truncation rejection
//=================================================================================================
void testTruncationRejection() {
  const fs::path complete = workDir() / "truncate-source.peinfo";
  pe::writeCheckpointMetadata(complete, sampleMetadata());

  // Cut the file in half: the surviving prefix parses line by line but is missing directives.
  const fs::path truncated = workDir() / "truncated.peinfo";
  {
    fs::ifstream in(complete);
    std::ostringstream all;
    all << in.rdbuf();
    const std::string text = all.str();
    fs::ofstream out(truncated, std::ios::out | std::ios::trunc);
    out << text.substr(0, text.size() / 2);
  }

  bool rejected = false;
  try {
    pe::readCheckpointMetadata(truncated);
  } catch (const std::exception& e) {
    rejected = true;
    std::cout << "  (expected throw for truncated sidecar: " << e.what() << ")" << std::endl;
  }
  check(rejected, "truncation: a half-written sidecar must be refused");

  // A sidecar that promises more materials than it contains is equally a truncation.
  const fs::path shortTable = workDir() / "short-material-table.peinfo";
  {
    pe::CheckpointMetadata metadata = sampleMetadata();
    pe::writeCheckpointMetadata(shortTable, metadata);
    fs::ifstream in(shortTable);
    std::ostringstream all;
    all << in.rdbuf();
    std::string text = all.str();
    const std::string needle = "materialCount 1";
    const std::string::size_type at = text.find(needle);
    check(at != std::string::npos, "truncation: fixture must contain a materialCount directive");
    if (at != std::string::npos) {
      text.replace(at, needle.size(), "materialCount 2");
    }
    fs::ofstream out(shortTable, std::ios::out | std::ios::trunc);
    out << text;
  }

  bool countRejected = false;
  try {
    pe::readCheckpointMetadata(shortTable);
  } catch (const std::exception& e) {
    countRejected = true;
    std::cout << "  (expected throw for short material table: " << e.what() << ")" << std::endl;
  }
  check(countRejected, "truncation: a short material table must be refused");

  // A sidecar from a future layout must not be reinterpreted under the current rules.
  const fs::path future = workDir() / "future-version.peinfo";
  {
    fs::ifstream in(complete);
    std::ostringstream all;
    all << in.rdbuf();
    std::string text = all.str();
    std::ostringstream current;
    current << "metadataVersion " << pe::checkpointMetadataVersion;
    std::ostringstream ahead;
    ahead << "metadataVersion " << (pe::checkpointMetadataVersion + 1u);
    const std::string::size_type at = text.find(current.str());
    if (at != std::string::npos) {
      text.replace(at, current.str().size(), ahead.str());
    }
    fs::ofstream out(future, std::ios::out | std::ios::trunc);
    out << text;
  }

  bool versionRejected = false;
  try {
    pe::readCheckpointMetadata(future);
  } catch (const std::exception& e) {
    versionRejected = true;
    std::cout << "  (expected throw for future layout version: " << e.what() << ")" << std::endl;
  }
  check(versionRejected, "truncation: a future layout version must be refused");

  // Corruption that is not truncation: a repeated directive is ambiguous, and a fractional value
  // for an integral field parses as its integer prefix and leaves the rest on the line.
  struct Corruption {
    const char* name;
    const char* find;
    const char* replaceWith;
  };
  const Corruption corruptions[] = {
      {"repeated directive", "timeStep 2400", "timeStep 2400\ntimeStep 9999"},
      {"fractional integer", "timeStep 2400", "timeStep 2400.5"},
      {"trailing content", "bodyCount 7", "bodyCount 7 8"},
  };

  for (size_t i = 0; i < sizeof(corruptions) / sizeof(corruptions[0]); ++i) {
    const fs::path corrupt = workDir() / (std::string("corrupt-") + std::to_string(i) + ".peinfo");
    {
      fs::ifstream in(complete);
      std::ostringstream all;
      all << in.rdbuf();
      std::string text = all.str();
      const std::string::size_type at = text.find(corruptions[i].find);
      check(at != std::string::npos,
            std::string("corruption fixture: anchor not found for ") + corruptions[i].name);
      if (at != std::string::npos) {
        text.replace(at, std::string(corruptions[i].find).size(), corruptions[i].replaceWith);
      }
      fs::ofstream out(corrupt, std::ios::out | std::ios::trunc);
      out << text;
    }

    bool corruptRejected = false;
    try {
      pe::readCheckpointMetadata(corrupt);
    } catch (const std::exception& e) {
      corruptRejected = true;
      std::cout << "  (expected throw for " << corruptions[i].name << ": " << e.what() << ")"
                << std::endl;
    }
    check(corruptRejected,
          std::string("corruption: '") + corruptions[i].name + "' must be refused");
  }
}

//=================================================================================================
// 5. Mismatch guard contract
//=================================================================================================
void testMismatchGuard() {
  const pe::CheckpointMetadata metadata = sampleMetadata();

  struct Case {
    const char* name;
    bool shouldThrow;
    pe::ResumeExpectation expectation;
  };

  std::vector<Case> cases;

  {
    Case c = {"no expectation", false, pe::ResumeExpectation()};
    cases.push_back(c);
  }
  {
    pe::ResumeExpectation e;
    e.hasStep = true;
    e.step = metadata.timeStep;
    Case c = {"matching step", false, e};
    cases.push_back(c);
  }
  {
    pe::ResumeExpectation e;
    e.hasStep = true;
    e.step = metadata.timeStep + 1u;
    Case c = {"step off by one", true, e};
    cases.push_back(c);
  }
  {
    pe::ResumeExpectation e;
    e.hasTime = true;
    e.time = metadata.simulationTime;
    Case c = {"exact time", false, e};
    cases.push_back(c);
  }
  {
    // Inside the default half-step window: the rounding drift a real driver exhibits.
    pe::ResumeExpectation e;
    e.hasTime = true;
    e.time = metadata.simulationTime + pe::real(0.4) * metadata.stepSize;
    Case c = {"time within tolerance", false, e};
    cases.push_back(c);
  }
  {
    pe::ResumeExpectation e;
    e.hasTime = true;
    e.time = metadata.simulationTime + pe::real(0.6) * metadata.stepSize;
    Case c = {"time outside tolerance", true, e};
    cases.push_back(c);
  }
  {
    pe::ResumeExpectation e;
    e.hasTag = true;
    e.tag = metadata.pairingTag;
    Case c = {"matching tag", false, e};
    cases.push_back(c);
  }
  {
    pe::ResumeExpectation e;
    e.hasTag = true;
    e.tag = metadata.pairingTag + "!";
    Case c = {"different tag", true, e};
    cases.push_back(c);
  }

  for (size_t i = 0; i < cases.size(); ++i) {
    bool didThrow = false;
    try {
      pe::validateResumeExpectation(metadata, cases[i].expectation, "guard.peb");
    } catch (const std::exception& e) {
      didThrow = true;
      std::cout << "  (throw for " << cases[i].name << ": " << e.what() << ")" << std::endl;
    }
    check(didThrow == cases[i].shouldThrow,
          std::string("mismatch guard: '") + cases[i].name + "' should " +
              (cases[i].shouldThrow ? "throw" : "not throw"));
  }

  // An expectation that cannot be checked must fail loudly rather than pass by default.
  pe::CheckpointMetadata legacy;
  pe::ResumeExpectation expectation;
  expectation.hasStep = true;
  expectation.step = 1u;
  bool legacyRejected = false;
  try {
    pe::validateResumeExpectation(legacy, expectation, "legacy.peb");
  } catch (const std::exception& e) {
    legacyRejected = true;
    std::cout << "  (expected throw for expectation on legacy checkpoint: " << e.what() << ")"
              << std::endl;
  }
  check(legacyRejected,
        "mismatch guard: an expectation on a legacy checkpoint must be refused, not dropped");

  // A zero step size would collapse the tolerance window to zero and reject every real resume on
  // a rounding difference. It must say so rather than present itself as a time mismatch.
  pe::CheckpointMetadata noStepSize = metadata;
  noStepSize.stepSize = pe::real(0);
  pe::ResumeExpectation exactTime;
  exactTime.hasTime = true;
  exactTime.time = noStepSize.simulationTime;
  bool stepSizeRejected = false;
  std::string stepSizeMessage;
  try {
    pe::validateResumeExpectation(noStepSize, exactTime, "zero-stepsize.peb");
  } catch (const std::exception& e) {
    stepSizeRejected = true;
    stepSizeMessage = e.what();
    std::cout << "  (expected throw for zero step size: " << e.what() << ")" << std::endl;
  }
  check(stepSizeRejected, "mismatch guard: a zero step size with a time expectation must throw");
  check(stepSizeMessage.find("step size") != std::string::npos,
        "mismatch guard: the zero-step-size error must name the step size, not read as a time "
        "mismatch");

  // A step expectation does not depend on the step size, so it must still work.
  pe::ResumeExpectation stepOnly;
  stepOnly.hasStep = true;
  stepOnly.step = noStepSize.timeStep;
  bool stepOnlyAccepted = true;
  try {
    pe::validateResumeExpectation(noStepSize, stepOnly, "zero-stepsize.peb");
  } catch (const std::exception&) {
    stepOnlyAccepted = false;
  }
  check(stepOnlyAccepted, "mismatch guard: a step expectation must not depend on the step size");
}

//=================================================================================================
// 4. Material table restore and `.peb` integrity (mutates process-global state; runs last)
//=================================================================================================
void testMaterialRestoreAndPebIntegrity() {
  const fs::path checkpoints = workDir() / "checkpoints";

  const size_t builtins = pe::Material::count();
  const pe::MaterialID custom =
      pe::createMaterial("restore-me", pe::real(1.5), pe::real(0.1), pe::real(0.05),
                         pe::real(0.05), pe::real(0.3), pe::real(300), pe::real(1e6),
                         pe::real(1e5), pe::real(2e5));
  check(static_cast<size_t>(custom) == builtins, "material restore: custom material index");

  pe::TimeStep::stepsize(pe::real(0.005));
  pe::setCheckpointIdentity(pe::real(12.0), 2400u, "ff-step-2400");

  pe::createSphere(1, pe::Vec3(0, 0, 0), pe::real(0.01), custom);
  pe::createSphere(2, pe::Vec3(1, 0, 0), pe::real(0.01), custom);

  pe::writeCheckpoint(checkpoints, "restore_seed");

  const fs::path peb = checkpoints / "restore_seed.peb";
  const fs::path sidecar = pe::checkpointMetadataPath(checkpoints, "restore_seed");
  check(fs::exists(peb), "material restore: checkpoint .peb was not written");
  check(fs::exists(sidecar), "material restore: metadata sidecar was not written");

  // Nothing may be left behind under either marker: `.tmp-` for the in-progress copies, `.prev-`
  // for the preserved previous sidecar, which must be gone once the new one is published.
  size_t scratchFiles = 0;
  for (fs::directory_iterator it(checkpoints), end; it != end; ++it) {
    const std::string name = it->path().string();
    if (name.find(".tmp-") != std::string::npos || name.find(".prev-") != std::string::npos) {
      ++scratchFiles;
      std::cerr << "  leftover scratch file: " << name << std::endl;
    }
  }
  check(scratchFiles == 0, "atomic write: scratch files were left behind");

  const pe::CheckpointMetadata written = pe::readCheckpointMetadata(sidecar);
  check(written.present, "material restore: sidecar should be present");
  check(written.timeSource == pe::checkpointTimeFromDriver,
        "material restore: driver identity was not recorded");
  check(written.timeStep == 2400u, "material restore: driver step was not recorded");
  check(written.pairingTag == "ff-step-2400", "material restore: pairing tag was not recorded");
  check(written.bodyCount == 2u, "material restore: body count was not recorded");
  check(written.pebBytes == static_cast<uint64_t>(fs::file_size(peb)),
        "material restore: recorded .peb size does not match the file on disk");
  check(written.materials.size() == builtins + 1,
        "material restore: full material table was not recorded");
  if (!written.materials.empty()) {
    check(written.materials.back().name == "restore-me",
          "material restore: custom material missing from the recorded table");
  }

  // Reinstating against the live table is a no-op and must not fail.
  bool reinstated = true;
  try {
    pe::restoreCheckpointMaterials(written, peb.string());
  } catch (const std::exception& e) {
    reinstated = false;
    std::cerr << "  restoreCheckpointMaterials threw: " << e.what() << std::endl;
  }
  check(reinstated, "material restore: reinstating an already-matching table must succeed");
  check(pe::Material::count() == builtins + 1,
        "material restore: reinstating must not duplicate materials");

  // Diverging properties under the same name must be refused, not silently accepted: body
  // masses were derived from the recorded density.
  pe::CheckpointMetadata drifted = written;
  drifted.materials.back().density += pe::real(1);
  bool driftRejected = false;
  try {
    pe::restoreCheckpointMaterials(drifted, peb.string());
  } catch (const std::exception& e) {
    driftRejected = true;
    std::cout << "  (expected throw for drifted material: " << e.what() << ")" << std::endl;
  }
  check(driftRejected, "material restore: a diverging material property must be refused");

  // A `.peb` shorter than its sidecar records must be refused before it is parsed.
  const fs::path shortDir = workDir() / "short-checkpoint";
  fs::create_directories(shortDir);
  fs::copy_file(sidecar, pe::checkpointMetadataPath(shortDir, "restore_seed"));
  {
    fs::ifstream in(peb, std::ios::binary);
    std::ostringstream all;
    all << in.rdbuf();
    const std::string bytes = all.str();
    fs::ofstream out(shortDir / "restore_seed.peb", std::ios::out | std::ios::binary | std::ios::trunc);
    out.write(bytes.data(), static_cast<std::streamsize>(bytes.size() / 2));
  }

  bool shortRejected = false;
  try {
    pe::readCheckpoint(shortDir, "restore_seed");
  } catch (const std::exception& e) {
    shortRejected = true;
    std::cout << "  (expected throw for truncated .peb: " << e.what() << ")" << std::endl;
  }
  check(shortRejected, "truncation: a short .peb must be refused, not half-read");

  // The case above is caught by the sidecar's pebBytes. The reader's own length check is what
  // protects a LEGACY checkpoint, which has no sidecar to compare against, so it needs its own
  // fixture -- otherwise the reader gate is never actually exercised.
  //
  // Three truncation depths, because the reader validates in three stages: the fixed header, the
  // offset table (whose extent depends on a process count read from the file), and the body
  // chunks. Cutting only at one depth would leave two gates untested.
  const std::string truncationDepths[] = {"header", "offset-table", "body"};
  std::string pebBytesRaw;
  {
    fs::ifstream in(peb, std::ios::binary);
    std::ostringstream all;
    all << in.rdbuf();
    pebBytesRaw = all.str();
  }
  const size_t fixedHeaderBytes = 9 + sizeof(uint32_t);
  const size_t cutLengths[] = {fixedHeaderBytes / 2, fixedHeaderBytes + 2,
                               pebBytesRaw.size() / 2};

  for (size_t i = 0; i < 3; ++i) {
    const fs::path dir = workDir() / ("legacy-truncated-" + truncationDepths[i]);
    fs::create_directories(dir);
    {
      fs::ofstream out(dir / "restore_seed.peb",
                       std::ios::out | std::ios::binary | std::ios::trunc);
      out.write(pebBytesRaw.data(), static_cast<std::streamsize>(cutLengths[i]));
    }
    check(!fs::exists(pe::checkpointMetadataPath(dir, "restore_seed")),
          "truncation: legacy fixture must have no sidecar");

    bool rejectedHere = false;
    try {
      pe::readCheckpoint(dir, "restore_seed");
    } catch (const std::exception& e) {
      rejectedHere = true;
      std::cout << "  (expected throw for legacy .peb truncated at the " << truncationDepths[i]
                << ": " << e.what() << ")" << std::endl;
    }
    check(rejectedHere, "truncation: a sidecar-less .peb truncated at the " + truncationDepths[i] +
                            " must be refused by the reader itself");
  }

  // A checkpoint whose sidecar is removed is exactly what a pre-metadata pe build produced. It
  // must still load -- loudly, but without failing -- so existing checkpoints stay usable.
  const fs::path legacyDir = workDir() / "legacy-checkpoint";
  fs::create_directories(legacyDir);
  fs::copy_file(peb, legacyDir / "restore_seed.peb");
  check(!fs::exists(pe::checkpointMetadataPath(legacyDir, "restore_seed")),
        "legacy: fixture must have no sidecar");

  bool legacyLoaded = true;
  pe::CheckpointMetadata legacyMetadata;
  try {
    legacyMetadata = pe::readCheckpoint(legacyDir, "restore_seed");
  } catch (const std::exception& e) {
    legacyLoaded = false;
    std::cerr << "  legacy readCheckpoint threw: " << e.what() << std::endl;
  }
  check(legacyLoaded, "legacy: a sidecar-less checkpoint must still load");
  check(!legacyMetadata.present, "legacy: a sidecar-less checkpoint must report absent metadata");
}

} // namespace

int main(int argc, char** argv) {
#if HAVE_MPI
  MPI_Init(&argc, &argv);
#else
  (void)argc;
  (void)argv;
#endif

  try {
    fs::create_directories(workDir());

    testRoundtrip();
    testLegacyAcceptance();
    testTruncationRejection();
    testMismatchGuard();
    testMaterialRestoreAndPebIntegrity();

    fs::remove_all(workDir());
  } catch (const std::exception& e) {
    std::cerr << "pe_checkpoint_metadata_test: unexpected exception: " << e.what() << std::endl;
#if HAVE_MPI
    MPI_Finalize();
#endif
    return EXIT_FAILURE;
  }

#if HAVE_MPI
  MPI_Finalize();
#endif

  if (failures != 0) {
    std::cerr << "pe_checkpoint_metadata_test: " << failures << " check(s) failed" << std::endl;
    return EXIT_FAILURE;
  }

  std::cout << "pe_checkpoint_metadata_test: PASS" << std::endl;
  return EXIT_SUCCESS;
}

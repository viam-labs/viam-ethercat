// viam:ethercat:gantry attribute parsing. The component itself needs a motor to depend on, so
// these tests stop at parse(): the attributes, and the kinematics file it reads on the way.

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include <boost/variant/get.hpp>
#include <viam/sdk/common/kinematics.hpp>
#include <viam/sdk/common/proto_value.hpp>
#include <viam/sdk/config/resource.hpp>

#include "ethercat/errors.hpp"
#include "test_harness.hpp"
#include "viam/module/gantry_axis.hpp"

using ethercat::Error;
using ethercat::servo::GantryAxis;
using viam::sdk::ProtoStruct;
using viam::sdk::ProtoValue;
using viam::sdk::ResourceConfig;

namespace {

ProtoStruct good_attrs() {
    ProtoStruct a;
    a.emplace("motor", ProtoValue(std::string("x-motor")));
    a.emplace("length_mm", ProtoValue(500.0));
    a.emplace("mm_per_rev", ProtoValue(40.0));
    return a;
}

ResourceConfig config_with(ProtoStruct attrs) {
    return ResourceConfig("gantry", "x-axis", "rdk", std::move(attrs), "rdk:component:gantry", GantryAxis::model());
}

// A file in the temp dir that disappears with the test, so a leftover never feeds the next run.
class TempFile {
   public:
    explicit TempFile(const std::string& contents) {
        path_ = (std::filesystem::temp_directory_path() / ("gantry_axis_test_" + std::to_string(::getpid()) + ".json")).string();
        std::ofstream out(path_, std::ios::binary);
        out << contents;
    }
    ~TempFile() {
        std::remove(path_.c_str());
    }
    TempFile(const TempFile&) = delete;
    TempFile& operator=(const TempFile&) = delete;
    const std::string& path() const {
        return path_;
    }

   private:
    std::string path_;
};

const char* const kSva = R"({"name":"x-axis","kinematic_param_type":"SVA","links":[],"joints":[]})";

}  // namespace

TEST("no kinematics_file: the axis has no kinematics") {
    const GantryAxis::Attributes attrs = GantryAxis::parse(config_with(good_attrs()));
    CHECK_EQ(attrs.motor, std::string("x-motor"));
    CHECK(attrs.kinematics.empty());
}

TEST("kinematics_file: the file's bytes are kept verbatim") {
    const TempFile file(kSva);
    ProtoStruct a = good_attrs();
    a.emplace("kinematics_file", ProtoValue(file.path()));
    const GantryAxis::Attributes attrs = GantryAxis::parse(config_with(std::move(a)));
    const std::string got(attrs.kinematics.begin(), attrs.kinematics.end());
    CHECK_EQ(got, std::string(kSva));
}

TEST("kinematics_file: a missing file is a config error, not a silent fixed frame") {
    ProtoStruct a = good_attrs();
    a.emplace("kinematics_file", ProtoValue(std::string("/nonexistent/gantry_axis_test.json")));
    CHECK_THROWS_MSG(GantryAxis::parse(config_with(std::move(a))), Error, "kinematics_file");
}

TEST("kinematics_file: an empty file is refused") {
    const TempFile file("");
    ProtoStruct a = good_attrs();
    a.emplace("kinematics_file", ProtoValue(file.path()));
    CHECK_THROWS_MSG(GantryAxis::parse(config_with(std::move(a))), Error, "is empty");
}

TEST("kinematics_file: must be a non-empty string") {
    ProtoStruct a = good_attrs();
    a.emplace("kinematics_file", ProtoValue(42.0));
    CHECK_THROWS_MSG(GantryAxis::parse(config_with(std::move(a))), Error, "non-empty string");
    ProtoStruct b = good_attrs();
    b.emplace("kinematics_file", ProtoValue(std::string()));
    CHECK_THROWS_MSG(GantryAxis::parse(config_with(std::move(b))), Error, "non-empty string");
}

TEST("validate names the motor as the only dependency") {
    const std::vector<std::string> deps = GantryAxis::validate(config_with(good_attrs()));
    CHECK_EQ(deps.size(), std::size_t{1});
    CHECK_EQ(deps[0], std::string("x-motor"));
}

TEST_MAIN()

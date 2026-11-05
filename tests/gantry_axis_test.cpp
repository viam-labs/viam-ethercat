// viam:ethercat:gantry attribute parsing. The component itself needs a motor to depend on, so
// these tests stop at parse(): the attributes, and the kinematics file it reads on the way.

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

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

const char* const kSva = R"({"name":"x-axis","kinematic_param_type":"SVA","links":[],"joints":[]})";

// A valid config, plus whatever `attrs` adds.
ResourceConfig config(ProtoStruct attrs = {}) {
    attrs.emplace("motor", ProtoValue(std::string("x-motor")));
    attrs.emplace("length_mm", ProtoValue(500.0));
    attrs.emplace("mm_per_rev", ProtoValue(40.0));
    return ResourceConfig("gantry", "x-axis", "rdk", std::move(attrs), "rdk:component:gantry", GantryAxis::model());
}

GantryAxis::Attributes parse_with_file(ProtoValue kinematics_file) {
    ProtoStruct a;
    a.emplace("kinematics_file", std::move(kinematics_file));
    return GantryAxis::parse(config(std::move(a)));
}

// Overwritten by every test that uses it, so a leftover from an earlier run is never read.
std::string temp_file(const std::string& contents) {
    const std::string path = (std::filesystem::temp_directory_path() / "gantry_axis_test.json").string();
    std::ofstream(path, std::ios::binary) << contents;
    return path;
}

}  // namespace

TEST("no kinematics_file: the axis has no kinematics") {
    const GantryAxis::Attributes attrs = GantryAxis::parse(config());
    CHECK_EQ(attrs.motor, std::string("x-motor"));
    CHECK(attrs.kinematics.empty());
}

TEST("kinematics_file: the file's bytes are kept verbatim") {
    const std::vector<unsigned char> k = parse_with_file(ProtoValue(temp_file(kSva))).kinematics;
    CHECK_EQ(std::string(k.begin(), k.end()), std::string(kSva));
}

TEST("kinematics_file: a missing or empty file is a config error, not a silent fixed frame") {
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(std::string("/nonexistent/gantry_axis_test.json"))), Error, "missing or empty");
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(temp_file(""))), Error, "missing or empty");
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(std::string())), Error, "missing or empty");
}

TEST("kinematics_file: must be a string") {
    CHECK_THROWS_MSG(parse_with_file(ProtoValue(42.0)), Error, "wrong type");
}

TEST("validate names the motor as the only dependency") {
    const std::vector<std::string> deps = GantryAxis::validate(config());
    CHECK_EQ(deps.size(), std::size_t{1});
    CHECK_EQ(deps[0], std::string("x-motor"));
}

TEST_MAIN()

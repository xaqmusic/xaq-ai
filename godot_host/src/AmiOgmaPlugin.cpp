// =============================================================================
// AmiOgmaPlugin.cpp  --  GDExtension entry point
// =============================================================================
//
// Registers OgmaBrain (and any future Ogma GDObjects) with Godot's ClassDB.

#include <gdextension_interface.h>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

#include "OgmaBrain.hpp"
#include "BenchClient.hpp"
#include "VideoClient.hpp"

using namespace godot;

static void initialize_ami_ogma_module(ModuleInitializationLevel p_level) {
    if (p_level != MODULE_INITIALIZATION_LEVEL_SCENE) return;
    ClassDB::register_class<OgmaBrain>();
    ClassDB::register_class<BenchClient>();
    ClassDB::register_class<VideoClient>();
}

static void uninitialize_ami_ogma_module(ModuleInitializationLevel p_level) {
    (void)p_level;
}

extern "C" {

GDExtensionBool GDE_EXPORT ami_ogma_library_init(
    GDExtensionInterfaceGetProcAddress p_get_proc_address,
    GDExtensionClassLibraryPtr         p_library,
    GDExtensionInitialization          *r_initialization
) {
    GDExtensionBinding::InitObject init_obj(p_get_proc_address, p_library, r_initialization);
    init_obj.register_initializer(initialize_ami_ogma_module);
    init_obj.register_terminator(uninitialize_ami_ogma_module);
    init_obj.set_minimum_library_initialization_level(MODULE_INITIALIZATION_LEVEL_SCENE);
    return init_obj.init();
}

} // extern "C"

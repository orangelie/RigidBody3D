cmake_minimum_required (VERSION 3.8)

# Build HLSL shaders (BEGIN)
add_custom_target(shaders)

set(HLSL_SHADER_FILES Shaders/OpaqueShader.hlsl)

set_source_files_properties(${HLSL_SHADER_FILES} PROPERTIES ShaderType "vs")
set_source_files_properties(${HLSL_SHADER_FILES} PROPERTIES ShaderType "ps")
set_source_files_properties(${HLSL_SHADER_FILES} PROPERTIES ShaderModel "5_1")

foreach(FILE ${HLSL_SHADER_FILES})
  get_filename_component(FILE_WE ${FILE} NAME_WE)
  get_source_file_property(shadertype ${FILE} ShaderType)
  get_source_file_property(shadermodel ${FILE} ShaderModel)
  add_custom_command(TARGET shaders
                     COMMAND fxc.exe /nologo /Emain /T${shadertype}_${shadermodel} $<IF:$<CONFIG:DEBUG>,/Od,/O1> /Zi /Fo ${CMAKE_BINARY_DIR}/Dx12UIFont@orangelie/${FILE_WE}.cso /Fd ${CMAKE_BINARY_DIR}/Dx12UIFont@orangelie/${FILE_WE}.pdb ${FILE}
                     MAIN_DEPENDENCY ${FILE}
                     COMMENT "HLSL ${FILE}"
                     WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
                     VERBATIM)
endforeach(FILE)
# Build HLSL shaders (END)


set(DEPENDENCIES "Dependencies/d3dx12.h" "Dependencies/DDSTextureLoader.cpp" "Dependencies/DDSTextureLoader.h")
set(UTILITIES "Utils.cpp" "Utils.h")
set(DXGI_RENDERER "Renderer.cpp" "Renderer.h")
set(GAMETIMER "GameTimer.h" "GameTimer.cpp")
set(GEOMETRYGENN "GeometryGenerator.h" "GeometryGenerator.cpp")
set(FRAMERESOURCE_DF "Test/FrameResource/RigidBodySim/FrameResource.h" "Test/FrameResource/RigidBodySim/FrameResource.cpp")
set(CAMERA "Camera.h" "Camera.cpp")
set(FONT "Font.h" "Font.cpp")

set(RIGIDBODY "Physics/RigidBody.cpp" "Physics/RigidBody.h")
set(CONTACT "Physics/Contact.cpp" "Physics/Contact.h")
set(PHYSICS ${RIGIDBODY} ${CONTACT})

add_executable (RigidBody3D "main.cpp" "Test/RigidBodySim.h" "UploadBuffer.h" ${DEPENDENCIES} ${UTILITIES} ${DXGI_RENDERER} ${GAMETIMER} ${GEOMETRYGENN} ${FRAMERESOURCE_DF} ${CAMERA} ${FONT} ${PHYSICS})
include_directories(RigidBody3D "./Dependencies")
include_directories(RigidBody3D "./Physics")
include_directories(RigidBody3D "./")
add_dependencies(RigidBody3D shaders)
target_compile_definitions(RigidBody3D PRIVATE RIGIDBODYSIM)



add_custom_command(TARGET RigidBody3D POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/RigidBody3D@orangelie/Shaders
      ${CMAKE_BINARY_DIR}/RigidBody3D@orangelie)

add_custom_command(TARGET RigidBody3D POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${CMAKE_SOURCE_DIR}/RigidBody3D@orangelie/Textures
      ${CMAKE_BINARY_DIR}/RigidBody3D@orangelie)
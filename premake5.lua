-- solution 
solution "rayrun"
	location "generated"
	configurations { "Debug", "Release" }
	platforms {"x64"}
	buildoptions { "/MP" }
	
-- 
project "rayrun"
	kind "ConsoleApp"
	language "C++"
	characterset "MBCS"
	files {
		"src/main.cpp",
		"src/rayrun.hpp",
	}
	includedirs {
		"thirdparty/stb/",
		"thirdparty/tinyobjloader/",
		"thirdparty/picojson/",
	}
	cppdialect "C++17"
	dependson { "refimp" }
	debugargs { "refimp", "../asset/hairball.json" }
	buildoptions { "/openmp" }
	filter "Release"
		optimize "Speed"

	filter {}

	-- 
project "refimp"
	kind "SharedLib"
	language "C++"
	characterset "MBCS"
	files {
		"src/refimpl.cpp",
		"src/rayrun.hpp",
	}
	cppdialect "C++17"
	filter "Release"
		optimize "Speed"

	filter {}
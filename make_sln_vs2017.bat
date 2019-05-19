pushd .
cd ..

if not exist BuildRayRun2019 (
    mkdir BuildRayRun2019
)

cd BuildRayRun2019
cmake ../rayrun2019 -G "Visual Studio 15 2017 Win64"

popd

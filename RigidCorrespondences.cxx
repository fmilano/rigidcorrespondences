/*
 * This file is part of the rigidcorrespondences library.
 *
 * Author: Federico Milano (fmilano@itba.edu.ar)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright
 * notice, this list of conditions and the following disclaimer in the
 * documentation and/or other materials provided with the distribution.
 *
 * Neither the name of the project's author nor the names of its
 * contributors may be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 * TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
 * PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#include <iostream>
#include <memory>
#include <cmath>

#include <vtkSmartPointer.h>
#include <vtkDirectory.h>
#include <vtkSTLReader.h>

using std::auto_ptr;


int getdir (std::string dir, std::vector<std::string> &files, const std::string& extension=".*")
{
	vtkSmartPointer<vtkDirectory> directory = vtkSmartPointer<vtkDirectory>::New();
	directory->Open(dir.c_str());

	for (unsigned i = 0; i < directory->GetNumberOfFiles(); i++) {
		const char* filename = directory->GetFile(i);
		if (extension == ".*" || std::string(filename).find(extension) != std::string::npos)
            files.push_back(filename);
	}
	directory->Delete();
    return 0;
}


vtkPolyData* loadVTKPolyData(const std::string& filename)
{
	vtkSTLReader* reader = vtkSTLReader::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkPolyData* pd = vtkPolyData::New();
	pd->ShallowCopy(reader->GetOutput());
	return pd;
}


//
// Change the datadir meshes correspondences according to the reference mesh.
// It assumed that the meshes are roughly aligned.
//
//
int main(int argc, char** argv) {

	if (argc < 3) {
		std::cout << "Usage " << argv[0] << " datadir referencename" << std::endl;
		return -1;
	}
	std::string datadir(argv[1]);
	std::string referenceName(argv[2]);



	typedef std::vector<std::string> StringVectorType;
	StringVectorType filenames;
    getdir(datadir, filenames, ".stl");
    if (filenames.size() == 0) {
        std::cerr << "did not find any stl files in directory " << datadir << " exiting.";
        return -1;
    }
	
	try {

	
        vtkSmartPointer<vtkPolyData> reference = loadVTKPolyData(datadir + "/" + referenceName);

		reference->ComputeBounds();
		double bounds[6];
		reference->GetBounds(bounds);


        // compute the maximum bound length to calculate the scaling factor
        double maxLength = 0;
        int maxBound = 0;
        for (int i = 0; i < 3; ++i) {
            double length = fabs(bounds[i*2] - bounds[i*2 + 1]);
            if (length > maxLength) {
                maxLength = length;
                maxBound = i;
            }
        }

		
        // rescale each mesh of the datadir (except the reference mesh) to the reference mesh scale
        for (int i = 0; i < filenames.size(); ++i) {

            if (filenames[i] == referenceName)
                continue;

            vtkSmartPointer<vtkPolyData> sample = loadVTKPolyData(datadir + "/" + filenames[i]);
			std::cout << datadir + "/" + filenames[i] << std::endl;

            sample->ComputeBounds();
            double sampleBounds[6];
            sample->GetBounds(sampleBounds);

            double sampleLength = fabs(sampleBounds[i*2] - sampleBounds[i*2 + 1]);
            double scale = maxLength/sampleLength;

		}

		// To actually build a model, we need to create a model builder object.
		// Calling the build model with a list of samples from the data manager, returns a new model.
		// The second parameter to BuildNewModel is the variance of the noise on our data
		auto_ptr<ModelBuilderType> modelBuilder(ModelBuilderType::Create());

		auto_ptr<StatisticalModelType> model(modelBuilder->BuildNewModel(dataManager->GetData(), 0.01));

		// Once we have built the model, we can save it to disk.
		model->Save(modelname);
		std::cout << "Successfully saved shape model as " << modelname << std::endl;

		reference->Delete();
	}
	catch (StatisticalModelException& e) {
		std::cout << "Exception occured while building the shape model" << std::endl;
		std::cout << e.what() << std::endl;
	}
}

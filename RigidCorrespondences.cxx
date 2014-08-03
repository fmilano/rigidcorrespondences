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
// Build a new shape model from vtkPolyData, given in datadir.
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
    	exit(-1);
  }
	
	try {

	
		vtkPolyData* reference = loadVTKPolyData(datadir + "/" + referenceName);

		reference->ComputeBounds();
		double bounds[6];
		reference->GetBounds(bounds);
		
		
		// Now we add our data to the data manager
		// load the data and add it to the data manager. We take the first 17 hand shapes that we find in the data folder
		for (unsigned i = 0; i < filenames.size() ; i++) {
			vtkPolyData* dataset = loadVTKPolyData(datadir + "/" + filenames[i]);
			std::cout << datadir + "/" + filenames[i] << std::endl;

			// We provde the filename as a second argument.
			// It will be written as metadata, and allows us to more easily figure out what we did later.
			dataManager->AddDataset(dataset, filenames[i]);

			// it is save to delete the dataset after it was added, as the datamanager direclty copies it.
			dataset->Delete();
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

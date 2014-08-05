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
#include <vtkSTLWriter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkPolyData.h>
#include <vtkTransform.h>
#include <vtkPolyDataNormals.h>
#include <vtkIterativeClosestPointTransform.h>
#include <vtkLandmarkTransform.h>
#include <vtkPointLocator.h>
#include <vtkCellLocator.h>
#include <vtkGenericCell.h>
#include <vtkIncrementalPointLocator.h>
#include <vtkIdList.h>
#include <vtkMath.h>

#include <vtkPolyDataReader.h>

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
    return 0;
}


vtkPolyData* loadVTKPolyData(const std::string& filename)
{
    vtkSmartPointer<vtkPolyDataReader> reader = vtkSmartPointer<vtkPolyDataReader>::New();
	reader->SetFileName(filename.c_str());
	reader->Update();
	vtkPolyData* pd = vtkPolyData::New();
	pd->ShallowCopy(reader->GetOutput());
	return pd;
}

vtkPolyData* loadSTLPolyData(const std::string& filename)
{
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(reader->GetOutput());
    return pd;
}

vtkPolyData* scalePolyData(vtkPolyData* polydata, double scale)
{
    vtkSmartPointer<vtkTransform> scaleTransform = vtkSmartPointer<vtkTransform>::New();
    scaleTransform->Scale(scale, scale, scale);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInput(polydata);
    transformFilter->SetTransform(scaleTransform);
    transformFilter->Update();

    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(transformFilter->GetOutput());
    return pd;
}

vtkPolyData* translatePolyData(vtkPolyData* polydata, double x, double y, double z)
{
    vtkSmartPointer<vtkTransform> translateTransform = vtkSmartPointer<vtkTransform>::New();
    translateTransform->Translate(x, y, z);

    vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
    transformFilter->SetInput(polydata);
    transformFilter->SetTransform(translateTransform);
    transformFilter->Update();

    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(transformFilter->GetOutput());
    return pd;
}

//
// Change the datadir meshes correspondences according to the reference mesh.
// It assumed that the meshes are roughly aligned.
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
        double referenceMaxLength = 0;
        int referenceMaxBound = 0;
        for (int i = 0; i < 3; ++i) {
            double length = fabs(bounds[i*2] - bounds[i*2 + 1]);
            if (length > referenceMaxLength) {
                referenceMaxLength = length;
                referenceMaxBound = i;
            }
        }
		
        // rescale each mesh of the datadir (except the reference mesh) to the reference mesh scale
        for (int i = 0; i < filenames.size(); ++i) {

            if (filenames[i] == referenceName)
                continue;

            vtkSmartPointer<vtkPolyData> sample = loadSTLPolyData(datadir + "/" + filenames[i]);
			std::cout << datadir + "/" + filenames[i] << std::endl;

            sample->ComputeBounds();
            double sampleBounds[6];
            sample->GetBounds(sampleBounds);

            // Here we use the assumption that all the samples are roughly aligned!
            double sampleLength = fabs(sampleBounds[referenceMaxBound*2] - sampleBounds[referenceMaxBound*2 + 1]);
            double scale = referenceMaxLength/sampleLength;

            double referenceCenter[3], center[3];
            reference->GetCenter(referenceCenter);

            vtkSmartPointer<vtkPolyData> scaledSample = scalePolyData(sample, scale);

            // Scaling changes the center. Get the new center.
            scaledSample->GetCenter(center);

            vtkSmartPointer<vtkPolyData> translatedSample = translatePolyData(scaledSample, referenceCenter[0] - center[0], referenceCenter[1] - center[1], referenceCenter[2] - center[2]);

            std::cout << referenceCenter[0] << ";" << referenceCenter[1] << ";" << referenceCenter[2] << std::endl;
            std::cout << center[0] << ";" << center[1] << ";" << center[2] << std::endl;

            //
            // Refine the registration with ICP
            //
            vtkSmartPointer<vtkIterativeClosestPointTransform> ICPTransform = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();

            ICPTransform->SetCheckMeanDistance(true);
            ICPTransform->SetMaximumMeanDistance(0.03);
            ICPTransform->SetMaximumNumberOfIterations(5);
            ICPTransform->SetMaximumNumberOfLandmarks(reference->GetNumberOfPoints());
            ICPTransform->GetLandmarkTransform()->SetModeToRigidBody();
            ICPTransform->StartByMatchingCentroidsOff();

            ICPTransform->SetSource(translatedSample);
            ICPTransform->SetTarget(reference);

            ICPTransform->Update();

            vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
            transformFilter->SetInput(translatedSample);
            transformFilter->SetTransform(ICPTransform);
            transformFilter->Update();

            vtkSmartPointer<vtkPolyData> registeredSample = transformFilter->GetOutput();

            vtkSmartPointer<vtkPointLocator> pointLocator = vtkSmartPointer<vtkPointLocator>::New();
            pointLocator->SetDataSet(registeredSample);
            pointLocator->BuildLocator();

    /*        vtkSmartPointer<vtkCellLocator> cellLocator = vtkSmartPointer<vtkCellLocator>::New();
            cellLocator->SetDataSet(registeredSample);
            cellLocator->BuildLocator();
*/
            vtkSmartPointer<vtkPointLocator> incrementalLocator = vtkSmartPointer<vtkPointLocator>::New();
            vtkSmartPointer<vtkPoints> newPoints = vtkSmartPointer<vtkPoints>::New();
            incrementalLocator->InitPointInsertion(newPoints, registeredSample->GetBounds(), reference->GetNumberOfPoints());


            vtkSmartPointer<vtkPolyData> remeshed = vtkSmartPointer<vtkPolyData>::New();
            remeshed->DeepCopy(reference);

            vtkSmartPointer<vtkPoints> points = remeshed->GetPoints();

            //bool firstPoint = true;
            for (vtkIdType j = 0; j < points->GetNumberOfPoints(); ++j) {
                double p[3];
                points->GetPoint(j, p);

                double closestPoint[3];

                /*vtkSmartPointer<vtkGenericCell> cell = vtkSmartPointer<vtkGenericCell>::New();
                vtkIdType cellId;
                int subId;
                double dist2;
                cellLocator->FindClosestPoint(p, closestPoint, cell.GetPointer(), cellId, subId, dist2);
*/
                vtkIdType closestPointId = pointLocator->FindClosestPoint(p);
                if (closestPointId == -1)
                {
                    std::cerr << "FAILURE to find closest point." << std::endl;
                    exit(1);
                }

                registeredSample->GetPoint(closestPointId, closestPoint);

                // Just for PARTIAL models
                /*if (sqrt(vtkMath::Distance2BetweenPoints(p, closestPoint)) > 15)
                {
                    closestPoint[0] = p[0];
                    closestPoint[1] = p[1];
                    closestPoint[2] = p[2];
                }*/

                if (incrementalLocator->IsInsertedPoint(closestPoint) > -1)
                {
                    int k = 2;
                    do
                    {
                        vtkSmartPointer<vtkIdList> idList = vtkSmartPointer<vtkIdList>::New();
                        idList->SetNumberOfIds(k);
                        pointLocator->FindClosestNPoints(k, p, idList);
                        if (idList->GetNumberOfIds() < k)
                        {
                            std::cerr << "FAILURE to find " << k << " closest points." << std::endl;
                            exit(1);
                        }

                        // computes the center of mass
                        closestPoint[0] = 0;
                        closestPoint[1] = 0;
                        closestPoint[2] = 0;

                        for (int h = 0; h < idList->GetNumberOfIds(); ++h)
                        {
                            double point[3];
                            registeredSample->GetPoint(idList->GetId(h), point);

                            closestPoint[0] += point[0];
                            closestPoint[1] += point[1];
                            closestPoint[2] += point[2];
                        }

                        closestPoint[0] /= k;
                        closestPoint[1] /= k;
                        closestPoint[2] /= k;

                        k++;
                    }
                    while (incrementalLocator->IsInsertedPoint(closestPoint) > -1);

                    if (k > 3)
                    {
                        std::cout << j;// << std::endl;
                        std::cout << " :(" << p[0] << "," << p[1] << "," << p[2] << ")-->(" << closestPoint[0] << "," << closestPoint[1] << "," << closestPoint[2] << ")" <<  std::endl;
                    }

                }

                incrementalLocator->InsertNextPoint(closestPoint);

                points->SetPoint(j, closestPoint);
                points->Modified();


            }


            remeshed->Modified();
            remeshed->Update();

            remeshed = scalePolyData(remeshed, sampleLength/referenceMaxLength);

            // Scaling changes the center. Get the new center.
            remeshed->GetCenter(center);
            remeshed = translatePolyData(remeshed, referenceCenter[0] - center[0], referenceCenter[1] - center[1], referenceCenter[2] - center[2]);

            size_t pos = filenames[i].find_last_of(".");
            filenames[i].insert(pos, "sr");

            vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
            writer->SetInput(remeshed);
            writer->SetFileTypeToBinary();
            writer->SetFileName((datadir + "/" + filenames[i]).c_str());
            writer->Update();

            std::cout << remeshed->GetNumberOfPoints() << std::endl;
		}

	}
    catch (std::exception& e) {
        std::cout << "Exception occured while calculating correspondences" << std::endl;
		std::cout << e.what() << std::endl;
	}
}


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

vtkPolyData* loadSTLPolyData(const std::string& filename)
{
    vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
    reader->SetFileName(filename.c_str());
    reader->Update();
    vtkPolyData* pd = vtkPolyData::New();
    pd->ShallowCopy(reader->GetOutput());
    return pd;
}


//
// Iterative closest points.
//
int main(int argc, char** argv) {

	if (argc < 4) {
		std::cout << "Usage " << argv[0] << " source target result" << std::endl;
		return -1;
	}
	std::string source(argv[1]);
	std::string target(argv[2]);
    std::string result(argv[3]);
	
	try {
	
        vtkSmartPointer<vtkPolyData> sourcepd = loadSTLPolyData(source);
        vtkSmartPointer<vtkPolyData> targetpd = loadSTLPolyData(target);

        vtkSmartPointer<vtkIterativeClosestPointTransform> ICPTransform = vtkSmartPointer<vtkIterativeClosestPointTransform>::New();

        ICPTransform->SetCheckMeanDistance(true);
        ICPTransform->SetMaximumMeanDistance(0.03);
        ICPTransform->SetMaximumNumberOfIterations(5);
        ICPTransform->SetMaximumNumberOfLandmarks(targetpd->GetNumberOfPoints());
        ICPTransform->GetLandmarkTransform()->SetModeToRigidBody();
        ICPTransform->StartByMatchingCentroidsOff();

        ICPTransform->SetSource(sourcepd);
        ICPTransform->SetTarget(targetpd);

        ICPTransform->Update();

        vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        transformFilter->SetInput(sourcepd);
        transformFilter->SetTransform(ICPTransform);
        transformFilter->Update();

        vtkSmartPointer<vtkSTLWriter> writer = vtkSmartPointer<vtkSTLWriter>::New();
        writer->SetInput(transformFilter->GetOutput());
        writer->SetFileTypeToBinary();
        writer->SetFileName(result.c_str());
        writer->Update();
	}
    catch (std::exception& e) {
        std::cout << "Exception occured while calculating correspondences" << std::endl;
		std::cout << e.what() << std::endl;
	}
}


//
// Copyright (c) 2017-2020 the rbfx project.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
// THE SOFTWARE.
//

#include <EASTL/fixed_vector.h>

#include "Gizmo.h"

namespace Urho3D
{

Gizmo::Gizmo(Context* context) : Object(context)
{
}

Gizmo::~Gizmo()
{
    UnsubscribeFromAllEvents();
}

bool Gizmo::IsActive() const
{
    return ImGuizmo::IsUsing();
}

bool Gizmo::ManipulateNode(const Camera* camera, Node* node)
{
    ea::fixed_vector<Node*, 1> nodes;
    nodes.push_back(node);
    return Manipulate(camera, nodes);
}

void Gizmo::RenderUI()
{
    ui::TextUnformatted("Op:");
    ui::SameLine(60);

    if (ui::RadioButton("Tr", GetOperation() == GIZMOOP_TRANSLATE))
        SetOperation(GIZMOOP_TRANSLATE);
    ui::SameLine();
    if (ui::RadioButton("Rot", GetOperation() == GIZMOOP_ROTATE))
        SetOperation(GIZMOOP_ROTATE);
    ui::SameLine();
    if (ui::RadioButton("Scl", GetOperation() == GIZMOOP_SCALE))
        SetOperation(GIZMOOP_SCALE);

    ui::TextUnformatted("Space:");
    ui::SameLine(60);
    if (ui::RadioButton("World", GetTransformSpace() == TS_WORLD))
        SetTransformSpace(TS_WORLD);
    ui::SameLine();
    if (ui::RadioButton("Local", GetTransformSpace() == TS_LOCAL))
        SetTransformSpace(TS_LOCAL);
}

}

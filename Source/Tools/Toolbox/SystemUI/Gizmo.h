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

#pragma once

#include "ToolboxAPI.h"
#include <Urho3D/Scene/Scene.h>
#include <Urho3D/Scene/Node.h>
#include <Urho3D/Graphics/Camera.h>
#include <Urho3D/IO/Log.h>
#include <Urho3D/SystemUI/SystemUI.h>
#include <Urho3D/SystemUI/SystemUIEvents.h>
#include <ImGuizmo/ImGuizmo.h>

namespace Urho3D
{

enum GizmoOperation
{
    GIZMOOP_TRANSLATE,
    GIZMOOP_ROTATE,
    GIZMOOP_SCALE,
    GIZMOOP_MAX
};

class URHO3D_TOOLBOX_API Gizmo : public Object
{
    URHO3D_OBJECT(Gizmo, Object);
public:
    /// Construct.
    explicit Gizmo(Context* context);
    /// Destruct.
    ~Gizmo() override;
    /// Manipulate node. Should be called from within E_UPDATE event.
    /// \param camera which observes the node.
    /// \param node to be manipulated.
    /// \returns true if node was manipulated on current frame.
    bool ManipulateNode(const Camera* camera, Node* node);
    /// Manipulate multiple nodes. Should be called from within E_UPDATE event.
    /// \param camera which observes the node.
    /// \param nodes to be manipulated. Specifying more than one node manipulates them in world space.
    /// \returns true if node was manipulated on current frame.
    template<typename Container>
    bool Manipulate(const Camera* camera, const Container& nodes);
    /// Set operation mode. Possible modes: rotation, translation and scaling.
    void SetOperation(GizmoOperation operation) { operation_ = operation; }
    /// Get current manipulation mode.
    GizmoOperation GetOperation() const { return operation_; };
    /// Set transform space in which gizmo should operate. Parent transform space is not supported.
    void SetTransformSpace(TransformSpace transformSpace) { transformSpace_ = transformSpace; }
    /// Get transform space in which gizmo is operating.
    TransformSpace GetTransformSpace() const { return transformSpace_; }
    /// Returns state of gizmo.
    /// \returns true if gizmo is active, i.e. mouse is held down.
    bool IsActive() const;
    /// Render gizmo ui. This needs to be called between ui::Begin() / ui::End().
    void RenderUI();
    /// Get the center of selected nodes.
    /// \param nodes The nodes to be calculated.
    /// \param outCenter If returns true, it gets the center, else it will be set to ZERO vector
    /// \returns Returns the number of selected nodes.
    template<typename Container>
    static int GetSelectionCenter(const Container& nodes, Vector3& outCenter);
    /// Get the center of selected nodes.

protected:
    /// Current gizmo operation. Translation, rotation or scaling.
    GizmoOperation operation_ = GIZMOOP_TRANSLATE;
    /// Current coordinate space to operate in. World or local.
    TransformSpace transformSpace_ = TS_WORLD;
    /// Saved node scale on operation start.
    ea::unordered_map<Node*, Vector3> nodeScaleStart_;
    /// Flag indicating that gizmo was active on the last frame.
    bool wasActive_ = false;
    /// A map of initial transforms.
    ea::unordered_map<Node*, Matrix3x4> initialTransforms_;
};

template<typename Container>
int Gizmo::GetSelectionCenter(const Container& nodes, Vector3& outCenter)
{
    outCenter = Vector3::ZERO;
    auto count = 0;
    for (const auto& node: nodes)
    {
        if (!node || node->GetType() == Scene::GetTypeStatic())
            continue;
        outCenter += node->GetWorldPosition();
        count++;
    }

    if (count != 0)
        outCenter /= count;
    return count;
}

template<typename Container>
bool Gizmo::Manipulate(const Camera* camera, const Container& nodes)
{
    if (nodes.empty())
        return false;

    ImGuizmo::SetOrthographic(camera->IsOrthographic());

    Matrix4 currentOrigin;
    if (!IsActive())
    {
        if (nodes.size() > 1)
        {
            // Find center point of all nodes
            // It is not clear what should be rotation and scale of center point for multiselection, therefore we limit
            // multiselection operations to world space (see above).
            Vector3 center;
            int count = GetSelectionCenter(nodes, center);
            if (count == 0)
                return false;
            currentOrigin.SetTranslation(center);
        }
        else if (!!*nodes.begin())
            currentOrigin = (*nodes.begin())->GetWorldTransform().ToMatrix4();
    }

    // Enums are compatible.
    auto operation = static_cast<ImGuizmo::OPERATION>(operation_);
    ImGuizmo::MODE mode = ImGuizmo::WORLD;
    // Scaling only works in local space. Multiselections only work in world space.
    if (transformSpace_ == TS_LOCAL)
        mode = ImGuizmo::LOCAL;
    else if (transformSpace_ == TS_WORLD)
        mode = ImGuizmo::WORLD;

    // Scaling is always done in local space even for multiselections.
    if (operation_ == GIZMOOP_SCALE)
        mode = ImGuizmo::LOCAL;
        // Any other operations on multiselections are done in world space.
    else if (nodes.size() > 1)
        mode = ImGuizmo::WORLD;

    Matrix4 view = camera->GetView().ToMatrix4().Transpose();
    Matrix4 proj = camera->GetProjection().Transpose();
    Matrix4 tran = currentOrigin.Transpose();
    Matrix4 delta;

    ImGuiWindow* window = ui::GetCurrentWindow();
    ImGuizmo::SetRect(window->Pos.x, window->Pos.y, window->Size.x, window->Size.y);
    ImGuizmo::Manipulate(&view.m00_, &proj.m00_, operation, mode, &tran.m00_, &delta.m00_, nullptr);

    if (IsActive())
    {
        if (!wasActive_)
        {
            // Just started modifying nodes.
            for (const auto& node: nodes)
                initialTransforms_[node] = node->GetTransform();
        }

        wasActive_ = true;
        tran = tran.Transpose();
        delta = delta.Transpose();

        currentOrigin = Matrix4(tran);

        for (const auto& node: nodes)
        {
            if (node == nullptr)
            {
                URHO3D_LOGERROR("Gizmo received null pointer of node.");
                continue;
            }

            if (operation_ == GIZMOOP_SCALE)
            {
                // A workaround for ImGuizmo bug where delta matrix returns absolute scale value.
                if (!nodeScaleStart_.contains(node))
                    nodeScaleStart_[node] = node->GetScale();
                node->SetScale(nodeScaleStart_[node] * delta.Scale());
            }
            else
            {
                // Delta matrix is always in world-space.
                if (operation_ == GIZMOOP_ROTATE)
                    node->RotateAround(currentOrigin.Translation(), -delta.Rotation(), TS_WORLD);
                else
                    node->Translate(delta.Translation(), TS_WORLD);
            }
        }

        return true;
    }
    else
    {
        if (wasActive_)
        {
            // Just finished modifying nodes.
            using namespace GizmoNodeModified;
            for (const auto& node: nodes)
            {
                if (!node)
                {
                    URHO3D_LOGWARNINGF("Node expired while manipulating it with gizmo.");
                    continue;
                }

                auto it = initialTransforms_.find(node);
                if (it == initialTransforms_.end())
                {
                    URHO3D_LOGWARNINGF("Gizmo has no record of initial node transform. List of transformed nodes "
                                       "changed mid-manipulation?");
                    continue;
                }

                SendEvent(E_GIZMONODEMODIFIED, P_NODE, &*node, P_OLDTRANSFORM, it->second,
                          P_NEWTRANSFORM, node->GetTransform());
            }
        }
        wasActive_ = false;
        initialTransforms_.clear();
        if (operation_ == GIZMOOP_SCALE && !nodeScaleStart_.empty())
            nodeScaleStart_.clear();
    }
    return false;
}

}

using System.Collections.Generic;
using Rhino.Geometry;

namespace NonPlanar_Robotic_Spatial_AM
{
#if SMT_AVAILABLE
    using System;
    using System.Linq;
    using Rhino;
    using Rhino.Commands;
    using SMT;
    using static SMT.SMTUtilities;

    /// <summary>
    /// Shared SMT writer service.
    /// Move the old WriteAllToSMT logic here so both planar and nonplanar pipelines can call the same exporter.
    /// Drop the SMT assemblies into libs\SMT\ to enable this path automatically.
    /// </summary>
    /// <remarks>
    /// This service exists to isolate the SMT dependency and the operation-writing side-effects from the slicers themselves.
    /// Unlike the core engines, these methods do modify Rhino and SMT UI state.
    /// </remarks>
    public static class SMTConnectionWriter
    {
        static SuperMatterToolsPlugin smtPlugin => SuperMatterToolsPlugin.Instance;

        /// <summary>
        /// Translates path curves into SMT point data and writes them into the selected SMT operation.
        /// </summary>
        /// <param name="allPathCurves">The ordered path curves to export. The value may be empty but should not be <see langword="null"/>.</param>
        /// <param name="verticalE5">The E5 value used for segments classified as vertical.</param>
        /// <param name="angledE5">The E5 value used for segments classified as angled.</param>
        /// <param name="horizontalE5">The E5 value used for segments classified as horizontal.</param>
        /// <param name="velocityRatioMultiplier">A positive multiplier used when mapping SMT velocity ratios.</param>
        /// <returns>
        /// An <see cref="SMTWriteResult"/> describing the generated planes and a log message. A failed setup returns a result
        /// with an explanatory message instead of throwing for ordinary operational issues.
        /// </returns>
        /// <remarks>
        /// Preconditions: SMT must be installed and a valid Rhino document should be active.
        /// Postconditions: when successful, the selected SMT operation contains newly written point data.
        /// Exceptions: unexpected SMT or Rhino API failures may still bubble up from underlying dependencies.
        /// Side-effects: modifies SMT UI state, creates user data, and selects the written operation or shape in the SMT UI.
        /// </remarks>
        public static SMTWriteResult WriteAllToSMT(
            IReadOnlyList<Curve> allPathCurves,
            double verticalE5,
            double angledE5,
            double horizontalE5,
            double velocityRatioMultiplier)
        {
            var generatedPlanes = new List<Plane>();

            if (allPathCurves == null || allPathCurves.Count == 0)
            {
                return new SMTWriteResult(generatedPlanes, "No curves were provided to the SMT writer.");
            }

            var setupResult = SMTSetup(RhinoDoc.ActiveDoc);
            if (setupResult != Result.Success)
            {
                return new SMTWriteResult(generatedPlanes, "SMT setup did not complete successfully.");
            }

            int progIndex = smtPlugin.UIData.ProgramIndex;
            int opIndex = smtPlugin.UIData.OperationIndex;
            if (progIndex < 0 || opIndex < 0)
            {
                return new SMTWriteResult(generatedPlanes, "You must select an SMT operation.");
            }

            OperationUI opUI = smtPlugin.UIData.TreeRootUI.WC.ChildNodes[progIndex].ChildNodes[opIndex];
            if (opUI == null)
            {
                return new SMTWriteResult(generatedPlanes, "The selected SMT operation is not available.");
            }

            ConfigureOperation(opUI);

            SuperEvent extrude = CreateEvent(opUI, "Extrude", EventType.Activate);
            SuperEvent stopExtrude = CreateEvent(opUI, "Extrude", EventType.Deactivate);
            SuperEvent heat = CreateEvent(opUI, "NozzleCooling", EventType.Activate);
            SuperEvent stopHeat = CreateEvent(opUI, "NozzleCooling", EventType.Deactivate);
            SuperEvent cool = CreateEvent(opUI, "NozzleCooling2", EventType.Activate);
            SuperEvent stopCooling = CreateEvent(opUI, "NozzleCooling2", EventType.Deactivate);
            SuperEvent cycleWait = CreateEvent(opUI, "CycleWait", EventType.Activate);

            int counter = 0;
            var allSMTPData = new List<List<SMTPData>>();

            for (int i = 0; i < allPathCurves.Count; i++)
            {
                Curve curve = allPathCurves[i];
                if (curve == null)
                {
                    continue;
                }

                if (!curve.TryGetPolyline(out Polyline polyline))
                {
                    polyline = BuildPolyline(curve);
                }

                if (polyline == null || polyline.Count < 2)
                {
                    continue;
                }

                var pathData = new List<SMTPData>();
                Plane? firstPathPlane = null;
                for (int segmentIndex = 0; segmentIndex < polyline.SegmentCount; segmentIndex++)
                {
                    Line line = polyline.SegmentAt(segmentIndex);
                    Plane plane = SMTPathMapper.CreatePathPlane(line);
                    firstPathPlane ??= plane;
                    generatedPlanes.Add(plane);

                    SegmentOrientationStyle style = SMTPathMapper.Classify(line);
                    double e5Value = SMTPathMapper.GetE5Value(style, verticalE5, angledE5, horizontalE5);
                    float velocityRatio = SMTPathMapper.GetVelocityRatio(style, velocityRatioMultiplier);

                    var smtpData = new SMTPData(counter++, MoveType.Lin, plane, velocityRatio);
                    smtpData.LengthParam = counter;
                    smtpData.MType = MoveType.Lin;
                    smtpData.AxisValues["E5"] = e5Value;

                    bool isFirstSegment = segmentIndex == 0;
                    bool isLastSegment = segmentIndex == polyline.SegmentCount - 1;

                    smtpData.Events["NozzleCooling2"] = style == SegmentOrientationStyle.Horizontal ? cool : stopCooling;
                    smtpData.Events["NozzleCooling"] = style == SegmentOrientationStyle.Vertical ? heat : stopHeat;
                    smtpData.Events["Extrude"] = isFirstSegment ? stopExtrude : extrude;

                    if (isLastSegment)
                    {
                        smtpData.Events["CycleWait"] = cycleWait;
                    }

                    pathData.Add(smtpData);
                }

                if (i > 0 && pathData.Count > 0 && firstPathPlane.HasValue)
                {
                    Curve previousCurve = allPathCurves[i - 1];
                    AddTraversalSequence(pathData, firstPathPlane.Value, previousCurve, ref counter, stopExtrude, stopCooling, stopHeat, generatedPlanes);
                }

                allSMTPData.Add(pathData);
            }

            WriteToOperation(opUI, allSMTPData);
            return new SMTWriteResult(generatedPlanes, $"Prepared {allSMTPData.Count} SMT path groups from {allPathCurves.Count} curves.");
        }

        private static void ConfigureOperation(OperationUI opUI)
        {
            opUI.DivStyle = DivisionStyle.PointData;
            opUI.FeedMode = FeedMapping.PointData;
            opUI.ZOrientationStyle = ZOrientStyle.PointData;
            opUI.YOrientationStyle = YOrientStyle.PointData;
            opUI.LIStyle = InOutStyle.Inactive;
            opUI.LOStyle = InOutStyle.Inactive;
            opUI.ActionControls["Extrude"].ActivationMode = ActivationStyle.PointData;
            opUI.ActionControls["NozzleCooling"].ActivationMode = ActivationStyle.PointData;
            opUI.ActionControls["NozzleCooling2"].ActivationMode = ActivationStyle.PointData;
            opUI.ActionControls["CycleWait"].ActivationMode = ActivationStyle.PointData;
            opUI.ActionControls["CycleWait"].StartValue = "3.5";
        }

        private static SuperEvent CreateEvent(OperationUI opUI, string actionName, EventType eventType)
        {
            ActionState action = opUI.SuperOperationRef.GetActionState(actionName);
            return new SuperEvent(action, 0.0, eventType, true);
        }

        private static void AddTraversalSequence(List<SMTPData> pathDataList, Plane nextPathStart, Curve previousCurve, ref int counter, SuperEvent stopExtrude, SuperEvent stopCooling, SuperEvent stopHeat, List<Plane> generatedPlanes)
        {
            if (previousCurve == null)
            {
                return;
            }

            Point3d previousEnd = previousCurve.PointAtEnd;
            if (previousEnd.DistanceTo(nextPathStart.Origin) <= 10.0)
            {
                return;
            }

            Plane endPlane = new Plane(previousEnd, nextPathStart.XAxis, nextPathStart.YAxis);
            var stopData = new SMTPData(counter++, MoveType.Lin, endPlane, 2.0f);
            stopData.Events["Extrude"] = stopExtrude;
            stopData.Events["NozzleCooling"] = stopHeat;
            stopData.Events["NozzleCooling2"] = stopCooling;

            Point3d liftPoint = new Point3d(previousEnd.X, previousEnd.Y, previousEnd.Z + 70.0);
            Plane liftPlane = new Plane(liftPoint, nextPathStart.XAxis, nextPathStart.YAxis);
            var liftData = new SMTPData(counter++, MoveType.Lin, liftPlane, 2.0f);

            Point3d traversePoint = new Point3d(nextPathStart.Origin.X, nextPathStart.Origin.Y, liftPoint.Z);
            Plane traversePlane = new Plane(traversePoint, nextPathStart.XAxis, nextPathStart.YAxis);
            var traverseData = new SMTPData(counter++, MoveType.Lin, traversePlane, 1.6f);
            traverseData.AxisValues["E5"] = 2.0;

            pathDataList.Insert(0, traverseData);
            pathDataList.Insert(0, liftData);
            pathDataList.Insert(0, stopData);

            generatedPlanes.Add(endPlane);
            generatedPlanes.Add(liftPlane);
            generatedPlanes.Add(traversePlane);
        }

        private static void WriteToOperation(OperationUI opUI, IReadOnlyList<List<SMTPData>> allSMTPData)
        {
            var shapes = new List<SuperShape>();

            foreach (List<SMTPData> pathData in allSMTPData)
            {
                Guid guid = Guid.NewGuid();
                smtPlugin.UserData[guid] = pathData.ToArray();
                SuperShape shape = SuperShape.SuperShapeFactory(guid, null, DivisionStyle.PointData, ZOrientStyle.PointData, VectorStyle.ByParam, YOrientStyle.PointData, false, 0.0, Plane.WorldXY);
                shapes.Add(shape);
            }

            if (shapes.Count == 0)
            {
                return;
            }

            var spbs = opUI.ReadFromGH(shapes.ToArray());
            if (spbs != null && spbs.Any())
            {
                spbs.Last().IsSelected = true;
                opUI.IsSelected = true;
            }
        }

        private static Polyline BuildPolyline(Curve curve)
        {
            double[] parameters = curve.DivideByCount(Math.Max(2, (int)Math.Ceiling(curve.GetLength() / 1.0)), true);
            if (parameters == null || parameters.Length < 2)
            {
                return new Polyline();
            }

            var points = parameters.Select(curve.PointAt).ToList();
            return new Polyline(points);
        }

        /// <summary>
        /// Ensures the minimal SMT workcell, program, and operation structure exists for writing point data.
        /// </summary>
        /// <param name="doc">The active Rhino document. It should not be <see langword="null"/>.</param>
        /// <returns>A Rhino <see cref="Result"/> indicating success, cancellation, or failure.</returns>
        /// <remarks>
        /// Preconditions: callers should supply the active Rhino document, and the document may need to be saved first.
        /// Postconditions: on success, the SMT UI is initialized and the default program and operation are selected.
        /// Exceptions: unexpected Rhino or SMT API failures may still bubble up.
        /// Side-effects: may save or prompt-save the Rhino file, starts the SMT UI, creates workcell data, and changes UI selection.
        /// </remarks>
        public static Result SMTSetup(RhinoDoc doc)
        {
            if (doc == null)
            {
                return Result.Failure;
            }

            if (doc.Name == string.Empty)
            {
                SaveRhinoFile(doc);
                doc = RhinoDoc.ActiveDoc;
            }

            if (doc == null || doc.Name == string.Empty)
            {
                return Result.Cancel;
            }

            SuperMatterToolsPlugin plugin = SuperMatterToolsPlugin.Instance;
            UIData uiData;
            if (plugin.UIData == null)
            {
                uiData = new UIData();
                uiData.Startup();
                plugin.UIData = uiData;
                plugin.UIData.PresentUI();
            }
            else
            {
                uiData = plugin.UIData;
            }

            if (uiData.TreeRootUI == null)
            {
                uiData.AddTreeRootUI("KR120_Solo");
            }

            var treeRoot = uiData.TreeRootUI!;
            if (treeRoot.WC.Name != "KR120_Solo")
            {
                treeRoot.WC.Name = "KR120_Solo";
            }

            if (treeRoot.WC.ChildNodes.Count() > 0)
            {
                treeRoot.WC.ChildNodes[0].Name = "Main";
            }
            else
            {
                treeRoot.WC.AddProgram("Main", 0);
            }

            ProgramUI programUI = treeRoot.WC.ChildNodes[0];
            if (programUI.ChildNodes.Count > 0)
            {
                programUI.ChildNodes[0].ProcessName = "Extruder_14mm22";
            }
            else
            {
                programUI.AddOperation(0, "StarExtruder_14mm_UofM");
            }

            uiData.PurgeUnusedUserData();
            programUI.SuperProgramRef.UpdateProgramData();
            uiData.UpdateLight();

            smtPlugin.UIData.TreeRootUI.WC.ChildNodes.First().IsSelected = true;
            smtPlugin.UIData.TreeRootUI.WC.ChildNodes.First().ChildNodes.First().IsSelected = true;
            return Result.Success;
        }
    }
#else
    /// <summary>
    /// Stub used until SMT assemblies are referenced. The slicer and Rhino/Grasshopper pieces can still compile.
    /// </summary>
    public static class SMTConnectionWriter
    {
        /// <summary>
        /// Returns a stub result explaining that SMT export is unavailable in the current build.
        /// </summary>
        /// <remarks>
        /// Differences: unlike the SMT-enabled implementation, this method never attempts to talk to Rhino or SMT and has no side-effects.
        /// </remarks>
        public static SMTWriteResult WriteAllToSMT(
            IReadOnlyList<Curve> allPathCurves,
            double verticalE5,
            double angledE5,
            double horizontalE5,
            double velocityRatioMultiplier)
        {
            return new SMTWriteResult(new List<Plane>(), "SMT assemblies are not referenced yet. Add the SMT DLLs under libs\\SMT\\ to enable export.");
        }
    }
#endif

    /// <summary>
    /// Describes the outcome of an SMT export attempt.
    /// </summary>
    public sealed class SMTWriteResult
    {
        /// <summary>
        /// Initializes a new SMT write result.
        /// </summary>
        /// <remarks>
        /// Preconditions: arguments should not be <see langword="null"/>.
        /// Postconditions: values are stored exactly as supplied.
        /// Exceptions: none in this constructor.
        /// Side-effects: none.
        /// </remarks>
        public SMTWriteResult(IReadOnlyList<Plane> generatedPlanes, string logMessage)
        {
            GeneratedPlanes = generatedPlanes;
            LogMessage = logMessage;
        }

        public IReadOnlyList<Plane> GeneratedPlanes { get; }
        public string LogMessage { get; }
    }
}

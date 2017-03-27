/*
 * Agent.cs
 * RVO2 Library C#
 *
 * Copyright (c) 2008-2015 University of North Carolina at Chapel Hill.
 * All rights reserved.
 *
 * Permission to use, copy, modify, and distribute this software and its
 * documentation for educational, research, and non-profit purposes, without
 * fee, and without a written agreement is hereby granted, provided that the
 * above copyright notice, this paragraph, and the following four paragraphs
 * appear in all copies.
 *
 * Permission to incorporate this software into commercial products may be
 * obtained by contacting the Office of Technology Development at the University
 * of North Carolina at Chapel Hill <otd@unc.edu>.
 *
 * This software program and documentation are copyrighted by the University of
 * North Carolina at Chapel Hill. The software program and documentation are
 * supplied "as is," without any accompanying services from the University of
 * North Carolina at Chapel Hill or the authors. The University of North
 * Carolina at Chapel Hill and the authors do not warrant that the operation of
 * the program will be uninterrupted or error-free. The end-user understands
 * that the program was developed for research purposes and is advised not to
 * rely exclusively on the program for any reason.
 *
 * IN NO EVENT SHALL THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL OR THE
 * AUTHORS BE LIABLE TO ANY PARTY FOR DIRECT, INDIRECT, SPECIAL, INCIDENTAL, OR
 * CONSEQUENTIAL DAMAGES, INCLUDING LOST PROFITS, ARISING OUT OF THE USE OF THIS
 * SOFTWARE AND ITS DOCUMENTATION, EVEN IF THE UNIVERSITY OF NORTH CAROLINA AT
 * CHAPEL HILL OR THE AUTHORS HAVE BEEN ADVISED OF THE POSSIBILITY OF SUCH
 * DAMAGE.
 *
 * THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE AUTHORS SPECIFICALLY
 * DISCLAIM ANY WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE AND ANY
 * STATUTORY WARRANTY OF NON-INFRINGEMENT. THE SOFTWARE PROVIDED HEREUNDER IS ON
 * AN "AS IS" BASIS, AND THE UNIVERSITY OF NORTH CAROLINA AT CHAPEL HILL AND THE
 * AUTHORS HAVE NO OBLIGATIONS TO PROVIDE MAINTENANCE, SUPPORT, UPDATES,
 * ENHANCEMENTS, OR MODIFICATIONS.
 *
 * Please send all bug reports to <geom@cs.unc.edu>.
 *
 * The authors may be contacted via:
 *
 * Jur van den Berg, Stephen J. Guy, Jamie Snape, Ming C. Lin, Dinesh Manocha
 * Dept. of Computer Science
 * 201 S. Columbia St.
 * Frederick P. Brooks, Jr. Computer Science Bldg.
 * Chapel Hill, N.C. 27599-3175
 * United States of America
 *
 * <http://gamma.cs.unc.edu/RVO2/>
 */

using System;
using System.Collections.Generic;

namespace RVO
{
    public enum AgentType
    {
        GroundUnit,//地面单位 可以移动 会和obstacle碰撞 会和GroundUnit碰撞 会和GroundJumpUnit碰撞 会和Building碰撞 会和敌人的SkillPush碰撞 会和SkillObstacle碰撞 会和SkillUnit碰撞
        GroundJumpUnit,//地面跳跃单位 可以移动 会和GroundUnit碰撞 会和GroundJumpUnit碰撞 会和Building碰撞 会和敌人的SkillPush碰撞 会和SkillObstacle碰撞 会和SkillUnit碰撞
        AirUnit,//空中单位 可以移动 会和AirUnit碰撞 会和敌人的SkillPush碰撞
        Building,//建筑 不可以移动
        SkillPush,//技能推力 不可以移动 每次移动结束后被删除
        SkillObstacle,//技能障碍 不可以移动
        SkillUnit,//技能单位 可以移动 会和obstacle碰撞 会和Building碰撞 会和SkillObstacle碰撞
    }

    /**
     * <summary>Defines an agent in the simulation.</summary>
     */
    public class Agent
    {
        private Simulator simulator;

        internal IList<KeyValuePair<double, Agent>> agentNeighbors_ = new List<KeyValuePair<double, Agent>>();
        internal IList<KeyValuePair<double, Obstacle>> obstacleNeighbors_ = new List<KeyValuePair<double, Obstacle>>();
        internal IList<Line> orcaLines_ = new List<Line>();
        internal Vector2 position_;
        internal Vector2 prefVelocity_;
        internal Vector2 velocity_;
        internal int maxNeighbors_ = 0;
        internal double maxSpeed_ = 0.0;
        internal double neighborDist_ = 0.0;
        internal double radius_ = 0.0;
        internal double timeHorizon_ = 0.0;
        internal double timeHorizonObst_ = 0.0;
        internal int weight;
        internal bool isMine;
        internal AgentType type;
        internal int uid;

        private bool bePush = false;
        private double beforePushSpeed;

        private Vector2 newVelocity_;

        internal Agent(Simulator _simulator)
        {
            simulator = _simulator;
        }

        /**
         * <summary>Computes the neighbors of this agent.</summary>
         */
        internal void computeNeighbors()
        {
            obstacleNeighbors_.Clear();
            double rangeSq = RVOMath.sqr(timeHorizonObst_ * maxSpeed_ + radius_);
            simulator.kdTree_.computeObstacleNeighbors(this, rangeSq);

            agentNeighbors_.Clear();

            if (maxNeighbors_ > 0)
            {
                rangeSq = RVOMath.sqr(neighborDist_);
                simulator.kdTree_.computeAgentNeighbors(this, ref rangeSq);
            }
        }

        /**
         * <summary>Computes the new velocity of this agent.</summary>
         */
        internal void computeNewVelocity()
        {
            orcaLines_.Clear();

            double invTimeHorizonObst = 1.0 / timeHorizonObst_;

            /* Create obstacle ORCA lines. */
            for (int i = 0; i < obstacleNeighbors_.Count; ++i)
            {

                Obstacle obstacle1 = obstacleNeighbors_[i].Value;
                Obstacle obstacle2 = obstacle1.next_;

                Vector2 relativePosition1 = obstacle1.point_ - position_;
                Vector2 relativePosition2 = obstacle2.point_ - position_;

                /*
                 * Check if velocity obstacle of obstacle is already taken care
                 * of by previously constructed obstacle ORCA lines.
                 */
                bool alreadyCovered = false;

                for (int j = 0; j < orcaLines_.Count; ++j)
                {
                    if (RVOMath.det(invTimeHorizonObst * relativePosition1 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON && RVOMath.det(invTimeHorizonObst * relativePosition2 - orcaLines_[j].point, orcaLines_[j].direction) - invTimeHorizonObst * radius_ >= -RVOMath.RVO_EPSILON)
                    {
                        alreadyCovered = true;

                        break;
                    }
                }

                if (alreadyCovered)
                {
                    continue;
                }

                /* Not yet covered. Check for collisions. */
                double distSq1 = RVOMath.absSq(relativePosition1);
                double distSq2 = RVOMath.absSq(relativePosition2);

                double radiusSq = RVOMath.sqr(radius_);

                Vector2 obstacleVector = obstacle2.point_ - obstacle1.point_;

                double s = Vector2.Dot(-relativePosition1, obstacleVector) / RVOMath.absSq(obstacleVector);
                double distSqLine = RVOMath.absSq(-relativePosition1 - s * obstacleVector);

                Line line;

                if (s < 0.0 && distSq1 <= radiusSq)
                {
                    /* Collision with left vertex. Ignore if non-convex. */
                    if (obstacle1.convex_)
                    {
                        line.point = new Vector2(0.0, 0.0);
                        line.direction = RVOMath.normalize(new Vector2(-relativePosition1.y, relativePosition1.x));
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s > 1.0 && distSq2 <= radiusSq)
                {
                    /*
                     * Collision with right vertex. Ignore if non-convex or if
                     * it will be taken care of by neighboring obstacle.
                     */
                    if (obstacle2.convex_ && RVOMath.det(relativePosition2, obstacle2.direction_) >= 0.0)
                    {
                        line.point = new Vector2(0.0, 0.0);
                        line.direction = RVOMath.normalize(new Vector2(-relativePosition2.y, relativePosition2.x));
                        orcaLines_.Add(line);
                    }

                    continue;
                }
                else if (s >= 0.0 && s < 1.0 && distSqLine <= radiusSq)
                {
                    /* Collision with obstacle segment. */
                    line.point = new Vector2(0.0, 0.0);
                    line.direction = -obstacle1.direction_;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * No collision. Compute legs. When obliquely viewed, both legs
                 * can come from a single vertex. Legs extend cut-off line when
                 * non-convex vertex.
                 */

                Vector2 leftLegDirection, rightLegDirection;

                if (s < 0.0 && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that left vertex
                     * defines velocity obstacle.
                     */
                    if (!obstacle1.convex_)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle2 = obstacle1;

                    double leg1 = Math.Sqrt(distSq1 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition1.x * leg1 - relativePosition1.y * radius_, relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
                    rightLegDirection = new Vector2(relativePosition1.x * leg1 + relativePosition1.y * radius_, -relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
                }
                else if (s > 1.0 && distSqLine <= radiusSq)
                {
                    /*
                     * Obstacle viewed obliquely so that
                     * right vertex defines velocity obstacle.
                     */
                    if (!obstacle2.convex_)
                    {
                        /* Ignore obstacle. */
                        continue;
                    }

                    obstacle1 = obstacle2;

                    double leg2 = Math.Sqrt(distSq2 - radiusSq);
                    leftLegDirection = new Vector2(relativePosition2.x * leg2 - relativePosition2.y * radius_, relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
                    rightLegDirection = new Vector2(relativePosition2.x * leg2 + relativePosition2.y * radius_, -relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
                }
                else
                {
                    /* Usual situation. */
                    if (obstacle1.convex_)
                    {
                        double leg1 = Math.Sqrt(distSq1 - radiusSq);
                        leftLegDirection = new Vector2(relativePosition1.x * leg1 - relativePosition1.y * radius_, relativePosition1.x * radius_ + relativePosition1.y * leg1) / distSq1;
                    }
                    else
                    {
                        /* Left vertex non-convex; left leg extends cut-off line. */
                        leftLegDirection = -obstacle1.direction_;
                    }

                    if (obstacle2.convex_)
                    {
                        double leg2 = Math.Sqrt(distSq2 - radiusSq);
                        rightLegDirection = new Vector2(relativePosition2.x * leg2 + relativePosition2.y * radius_, -relativePosition2.x * radius_ + relativePosition2.y * leg2) / distSq2;
                    }
                    else
                    {
                        /* Right vertex non-convex; right leg extends cut-off line. */
                        rightLegDirection = obstacle1.direction_;
                    }
                }

                /*
                 * Legs can never point into neighboring edge when convex
                 * vertex, take cutoff-line of neighboring edge instead. If
                 * velocity projected on "foreign" leg, no constraint is added.
                 */

                Obstacle leftNeighbor = obstacle1.previous_;

                bool isLeftLegForeign = false;
                bool isRightLegForeign = false;

                if (obstacle1.convex_ && RVOMath.det(leftLegDirection, -leftNeighbor.direction_) >= 0.0)
                {
                    /* Left leg points into obstacle. */
                    leftLegDirection = -leftNeighbor.direction_;
                    isLeftLegForeign = true;
                }

                if (obstacle2.convex_ && RVOMath.det(rightLegDirection, obstacle2.direction_) <= 0.0)
                {
                    /* Right leg points into obstacle. */
                    rightLegDirection = obstacle2.direction_;
                    isRightLegForeign = true;
                }

                /* Compute cut-off centers. */
                Vector2 leftCutOff = invTimeHorizonObst * (obstacle1.point_ - position_);
                Vector2 rightCutOff = invTimeHorizonObst * (obstacle2.point_ - position_);
                Vector2 cutOffVector = rightCutOff - leftCutOff;

                /* Project current velocity on velocity obstacle. */

                /* Check if current velocity is projected on cutoff circles. */
                double t = obstacle1 == obstacle2 ? 0.5 : (Vector2.Dot((velocity_ - leftCutOff), cutOffVector)) / RVOMath.absSq(cutOffVector);
                double tLeft = Vector2.Dot((velocity_ - leftCutOff), leftLegDirection);
                double tRight = Vector2.Dot((velocity_ - rightCutOff), rightLegDirection);

                if ((t < 0.0 && tLeft < 0.0) || (obstacle1 == obstacle2 && tLeft < 0.0 && tRight < 0.0))
                {
                    /* Project on left cut-off circle. */
                    Vector2 unitW = RVOMath.normalize(velocity_ - leftCutOff);

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }
                else if (t > 1.0 && tRight < 0.0)
                {
                    /* Project on right cut-off circle. */
                    Vector2 unitW = RVOMath.normalize(velocity_ - rightCutOff);

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    line.point = rightCutOff + radius_ * invTimeHorizonObst * unitW;
                    orcaLines_.Add(line);

                    continue;
                }

                /*
                 * Project on left leg, right leg, or cut-off line, whichever is
                 * closest to velocity.
                 */
                double distSqCutoff = (t < 0.0 || t > 1.0 || obstacle1 == obstacle2) ? double.PositiveInfinity : RVOMath.absSq(velocity_ - (leftCutOff + t * cutOffVector));
                double distSqLeft = tLeft < 0.0 ? double.PositiveInfinity : RVOMath.absSq(velocity_ - (leftCutOff + tLeft * leftLegDirection));
                double distSqRight = tRight < 0.0 ? double.PositiveInfinity : RVOMath.absSq(velocity_ - (rightCutOff + tRight * rightLegDirection));

                if (distSqCutoff <= distSqLeft && distSqCutoff <= distSqRight)
                {
                    /* Project on cut-off line. */
                    line.direction = -obstacle1.direction_;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y, line.direction.x);
                    orcaLines_.Add(line);

                    continue;
                }

                if (distSqLeft <= distSqRight)
                {
                    /* Project on left leg. */
                    if (isLeftLegForeign)
                    {
                        continue;
                    }

                    line.direction = leftLegDirection;
                    line.point = leftCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y, line.direction.x);
                    orcaLines_.Add(line);

                    continue;
                }

                /* Project on right leg. */
                if (isRightLegForeign)
                {
                    continue;
                }

                line.direction = -rightLegDirection;
                line.point = rightCutOff + radius_ * invTimeHorizonObst * new Vector2(-line.direction.y, line.direction.x);
                orcaLines_.Add(line);
            }

            int numObstLines = orcaLines_.Count;

            double invTimeHorizon = 1.0 / timeHorizon_;

            /* Create agent ORCA lines. */
            for (int i = 0; i < agentNeighbors_.Count; ++i)
            {
                Agent other = agentNeighbors_[i].Value;

                Vector2 relativePosition = other.position_ - position_;
                Vector2 relativeVelocity = velocity_ - other.velocity_;
                double distSq = RVOMath.absSq(relativePosition);
                double combinedRadius = radius_ + other.radius_;
                double combinedRadiusSq = RVOMath.sqr(combinedRadius);

                Line line;
                Vector2 u;

                if (distSq > combinedRadiusSq)
                {
                    /* No collision. */
                    Vector2 w = relativeVelocity - invTimeHorizon * relativePosition;

                    /* Vector from cutoff center to relative velocity. */
                    double wLengthSq = RVOMath.absSq(w);
                    double dotProduct1 = Vector2.Dot(w, relativePosition);

                    if (dotProduct1 < 0.0 && RVOMath.sqr(dotProduct1) > combinedRadiusSq * wLengthSq)
                    {
                        /* Project on cut-off circle. */
                        double wLength = Math.Sqrt(wLengthSq);
                        Vector2 unitW = w / wLength;

                        line.direction = new Vector2(unitW.y, -unitW.x);
                        u = (combinedRadius * invTimeHorizon - wLength) * unitW;
                    }
                    else
                    {
                        /* Project on legs. */
                        double leg = Math.Sqrt(distSq - combinedRadiusSq);

                        if (RVOMath.det(relativePosition, w) > 0.0)
                        {
                            /* Project on left leg. */
                            line.direction = new Vector2(relativePosition.x * leg - relativePosition.y * combinedRadius, relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }
                        else
                        {
                            /* Project on right leg. */
                            line.direction = -new Vector2(relativePosition.x * leg + relativePosition.y * combinedRadius, -relativePosition.x * combinedRadius + relativePosition.y * leg) / distSq;
                        }

                        double dotProduct2 = Vector2.Dot(relativeVelocity, line.direction);
                        u = dotProduct2 * line.direction - relativeVelocity;
                    }
                }
                else
                {
                    /* Collision. Project on cut-off circle of time timeStep. */
                    double invTimeStep = 1.0 / simulator.getTimeStep();

                    /* Vector from cutoff center to relative velocity. */
                    Vector2 w = relativeVelocity - invTimeStep * relativePosition;

                    double wLength = RVOMath.abs(w);
                    Vector2 unitW = w / wLength;

                    line.direction = new Vector2(unitW.y, -unitW.x);
                    u = (combinedRadius * invTimeStep - wLength) * unitW;
                }

                line.point = velocity_ + 0.5 * u;
                orcaLines_.Add(line);
            }

            int lineFail = linearProgram2(orcaLines_, maxSpeed_, prefVelocity_, false, ref newVelocity_);

            if (lineFail < orcaLines_.Count)
            {
                linearProgram3(orcaLines_, numObstLines, lineFail, maxSpeed_, ref newVelocity_);
            }
        }

        /**
         * <summary>Inserts an agent neighbor into the set of neighbors of this
         * agent.</summary>
         *
         * <param name="agent">A pointer to the agent to be inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertAgentNeighbor(Agent agent, ref double rangeSq)
        {
            if (this != agent)
            {
                if (agent.type == AgentType.SkillPush)
                {
                    if (agent.isMine == isMine || type == AgentType.SkillUnit)
                    {
                        return;
                    }
                }
                else if (agent.type == AgentType.Building || agent.type == AgentType.SkillObstacle)
                {
                    if (type == AgentType.AirUnit)
                    {
                        return;
                    }
                }
                else if (agent.type == AgentType.SkillUnit)
                {
                    if (type == AgentType.AirUnit || type == AgentType.SkillUnit)
                    {
                        return;
                    }
                }
                else
                {
                    if (type == AgentType.SkillUnit)
                    {
                        return;
                    }

                    if ((type == AgentType.AirUnit) != (agent.type == AgentType.AirUnit))
                    {
                        return;
                    }

                    if (weight > agent.weight)
                    {
                        return;
                    }

                    //if (!prefVelocity_.Equals(Vector2.zero) && agent.isMine != isMine)
                    //{
                    //    return;
                    //}
                }

                double distSq = RVOMath.absSq(position_ - agent.position_);

                if (distSq < rangeSq)
                {
                    if (agent.type == AgentType.SkillPush || agent.type == AgentType.Building || agent.type == AgentType.SkillObstacle || agent.type == AgentType.SkillUnit)
                    {
                        double dis = Math.Sqrt(distSq);

                        double maxDis = radius_ + agent.radius_;

                        if (maxDis > dis)
                        {
                            double newSpeed = (maxDis - dis) / simulator.getTimeStep();

                            if (!bePush)
                            {
                                agent.prefVelocity_ = Vector2.zero;

                                bePush = true;

                                beforePushSpeed = maxSpeed_;
                            }

                            if (newSpeed > maxSpeed_)
                            {
                                maxSpeed_ = newSpeed;
                            }
                        }
                    }

                    if (agentNeighbors_.Count < maxNeighbors_)
                    {
                        agentNeighbors_.Add(new KeyValuePair<double, Agent>(distSq, agent));
                    }

                    int i = agentNeighbors_.Count - 1;

                    while (i != 0 && distSq < agentNeighbors_[i - 1].Key)
                    {
                        agentNeighbors_[i] = agentNeighbors_[i - 1];
                        --i;
                    }

                    agentNeighbors_[i] = new KeyValuePair<double, Agent>(distSq, agent);

                    if (agentNeighbors_.Count == maxNeighbors_)
                    {
                        rangeSq = agentNeighbors_[agentNeighbors_.Count - 1].Key;
                    }
                }
            }
        }

        /**
         * <summary>Inserts a static obstacle neighbor into the set of neighbors
         * of this agent.</summary>
         *
         * <param name="obstacle">The number of the static obstacle to be
         * inserted.</param>
         * <param name="rangeSq">The squared range around this agent.</param>
         */
        internal void insertObstacleNeighbor(Obstacle obstacle, double rangeSq)
        {
            if ((type == AgentType.AirUnit || type == AgentType.GroundJumpUnit) && !obstacle.isMapBound)
            {
                return;
            }

            Obstacle nextObstacle = obstacle.next_;

            double distSq = RVOMath.distSqPointLineSegment(obstacle.point_, nextObstacle.point_, position_);

            if (distSq < rangeSq)
            {
                obstacleNeighbors_.Add(new KeyValuePair<double, Obstacle>(distSq, obstacle));

                int i = obstacleNeighbors_.Count - 1;

                while (i != 0 && distSq < obstacleNeighbors_[i - 1].Key)
                {
                    obstacleNeighbors_[i] = obstacleNeighbors_[i - 1];
                    --i;
                }
                obstacleNeighbors_[i] = new KeyValuePair<double, Obstacle>(distSq, obstacle);
            }
        }

        /**
         * <summary>Updates the two-dimensional position and two-dimensional
         * velocity of this agent.</summary>
         */
        internal void update()
        {
            velocity_ = newVelocity_;
            position_ += velocity_ * simulator.getTimeStep();

            if (bePush)
            {
                bePush = false;

                maxSpeed_ = beforePushSpeed;
            }
        }

        internal void checkPosInMapBound()
        {
            double radioFix = radius_ * simulator.getMapBoundFix();

            if (position_.x - radioFix < simulator.mapBounds.xMin)
            {
                double fix = (simulator.mapBounds.xMin - (position_.x - radioFix)) / simulator.mapBounds.width;

                position_ = new Vector2(simulator.mapBounds.xMin + radioFix - fix, position_.y);
            }
            else if (position_.x + radioFix > simulator.mapBounds.xMax)
            {
                double fix = (position_.x + radioFix - simulator.mapBounds.xMax) / simulator.mapBounds.width;

                position_ = new Vector2(simulator.mapBounds.xMax - radioFix + fix, position_.y);
            }

            if (position_.y - radioFix < simulator.mapBounds.yMin)
            {
                double fix = (simulator.mapBounds.yMin - (position_.y - radioFix)) / simulator.mapBounds.height;

                position_ = new Vector2(position_.x, simulator.mapBounds.yMin + radioFix - fix);
            }
            else if (position_.y + radioFix > simulator.mapBounds.yMax)
            {
                double fix = (position_.y + radioFix - simulator.mapBounds.yMax) / simulator.mapBounds.height;

                position_ = new Vector2(position_.x, simulator.mapBounds.yMax - radioFix + fix);
            }
        }

        internal void FixPosAndVelocity()
        {
            //velocity_ = new Vector2(Math.Round(velocity_.x, 4), Math.Round(velocity_.y, 4));
            //position_ = new Vector2(Math.Round(position_.x, 4), Math.Round(position_.y, 4));

            velocity_ = new Vector2(double.Parse(velocity_.x.ToString("F4")), double.Parse(velocity_.y.ToString("F4")));
            position_ = new Vector2(double.Parse(position_.x.ToString("F4")), double.Parse(position_.y.ToString("F4")));
        }

        /**
         * <summary>Solves a one-dimensional linear program on a specified line
         * subject to linear constraints defined by lines and a circular
         * constraint.</summary>
         *
         * <returns>True if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="lineNo">The specified line constraint.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private bool linearProgram1(IList<Line> lines, int lineNo, double radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            double dotProduct = Vector2.Dot(lines[lineNo].point, lines[lineNo].direction);
            double discriminant = RVOMath.sqr(dotProduct) + RVOMath.sqr(radius) - RVOMath.absSq(lines[lineNo].point);

            if (discriminant < 0.0)
            {
                /* Max speed circle fully invalidates line lineNo. */
                return false;
            }

            double sqrtDiscriminant = Math.Sqrt(discriminant);
            double tLeft = -dotProduct - sqrtDiscriminant;
            double tRight = -dotProduct + sqrtDiscriminant;

            for (int i = 0; i < lineNo; ++i)
            {
                double denominator = RVOMath.det(lines[lineNo].direction, lines[i].direction);
                double numerator = RVOMath.det(lines[i].direction, lines[lineNo].point - lines[i].point);

                if (Math.Abs(denominator) <= RVOMath.RVO_EPSILON)
                {
                    /* Lines lineNo and i are (almost) parallel. */
                    if (numerator < 0.0)
                    {
                        return false;
                    }

                    continue;
                }

                double t = numerator / denominator;

                if (denominator >= 0.0)
                {
                    /* Line i bounds line lineNo on the right. */
                    tRight = Math.Min(tRight, t);
                }
                else
                {
                    /* Line i bounds line lineNo on the left. */
                    tLeft = Math.Max(tLeft, t);
                }

                if (tLeft > tRight)
                {
                    return false;
                }
            }

            if (directionOpt)
            {
                /* Optimize direction. */
                if (Vector2.Dot(optVelocity, lines[lineNo].direction) > 0.0)
                {
                    /* Take right extreme. */
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    /* Take left extreme. */
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
            }
            else
            {
                /* Optimize closest point. */
                double t = Vector2.Dot(lines[lineNo].direction, (optVelocity - lines[lineNo].point));

                if (t < tLeft)
                {
                    result = lines[lineNo].point + tLeft * lines[lineNo].direction;
                }
                else if (t > tRight)
                {
                    result = lines[lineNo].point + tRight * lines[lineNo].direction;
                }
                else
                {
                    result = lines[lineNo].point + t * lines[lineNo].direction;
                }
            }

            return true;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <returns>The number of the line it fails on, and the number of lines
         * if successful.</returns>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="optVelocity">The optimization velocity.</param>
         * <param name="directionOpt">True if the direction should be optimized.
         * </param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private int linearProgram2(IList<Line> lines, double radius, Vector2 optVelocity, bool directionOpt, ref Vector2 result)
        {
            if (directionOpt)
            {
                /*
                 * Optimize direction. Note that the optimization velocity is of
                 * unit length in this case.
                 */
                result = optVelocity * radius;
            }
            else if (RVOMath.absSq(optVelocity) > RVOMath.sqr(radius))
            {
                /* Optimize closest point and outside circle. */
                result = RVOMath.normalize(optVelocity) * radius;
            }
            else
            {
                /* Optimize closest point and inside circle. */
                result = optVelocity;
            }

            for (int i = 0; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > 0.0)
                {
                    /* Result does not satisfy constraint i. Compute new optimal result. */
                    Vector2 tempResult = result;
                    if (!linearProgram1(lines, i, radius, optVelocity, directionOpt, ref result))
                    {
                        result = tempResult;

                        return i;
                    }
                }
            }

            return lines.Count;
        }

        /**
         * <summary>Solves a two-dimensional linear program subject to linear
         * constraints defined by lines and a circular constraint.</summary>
         *
         * <param name="lines">Lines defining the linear constraints.</param>
         * <param name="numObstLines">Count of obstacle lines.</param>
         * <param name="beginLine">The line on which the 2-d linear program
         * failed.</param>
         * <param name="radius">The radius of the circular constraint.</param>
         * <param name="result">A reference to the result of the linear program.
         * </param>
         */
        private void linearProgram3(IList<Line> lines, int numObstLines, int beginLine, double radius, ref Vector2 result)
        {
            double distance = 0.0;

            for (int i = beginLine; i < lines.Count; ++i)
            {
                if (RVOMath.det(lines[i].direction, lines[i].point - result) > distance)
                {
                    /* Result does not satisfy constraint of line i. */
                    IList<Line> projLines = new List<Line>();
                    for (int ii = 0; ii < numObstLines; ++ii)
                    {
                        projLines.Add(lines[ii]);
                    }

                    for (int j = numObstLines; j < i; ++j)
                    {
                        Line line;

                        double determinant = RVOMath.det(lines[i].direction, lines[j].direction);

                        if (Math.Abs(determinant) <= RVOMath.RVO_EPSILON)
                        {
                            /* Line i and line j are parallel. */
                            if (Vector2.Dot(lines[i].direction, lines[j].direction) > 0.0)
                            {
                                /* Line i and line j point in the same direction. */
                                continue;
                            }
                            else
                            {
                                /* Line i and line j point in opposite direction. */
                                line.point = 0.5 * (lines[i].point + lines[j].point);
                            }
                        }
                        else
                        {
                            line.point = lines[i].point + (RVOMath.det(lines[j].direction, lines[i].point - lines[j].point) / determinant) * lines[i].direction;
                        }

                        line.direction = RVOMath.normalize(lines[j].direction - lines[i].direction);
                        projLines.Add(line);
                    }

                    Vector2 tempResult = result;
                    if (linearProgram2(projLines, radius, new Vector2(-lines[i].direction.y, lines[i].direction.x), true, ref result) < projLines.Count)
                    {
                        /*
                         * This should in principle not happen. The result is by
                         * definition already in the feasible region of this
                         * linear program. If it fails, it is due to small
                         * doubleing point error, and the current result is kept.
                         */
                        result = tempResult;
                    }

                    distance = RVOMath.det(lines[i].direction, lines[i].point - result);
                }
            }
        }
    }
}

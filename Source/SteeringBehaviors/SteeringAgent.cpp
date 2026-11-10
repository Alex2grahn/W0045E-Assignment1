#include "SteeringAgent.h"
#include "DrawDebugHelpers.h"
#include "Math/UnrealMathUtility.h"

ASteeringAgent::ASteeringAgent()
{
    PrimaryActorTick.bCanEverTick = true;
}

void ASteeringAgent::BeginPlay()
{
    Super::BeginPlay();
}


void ASteeringAgent::Tick(float DeltaTime)
{
    Super::Tick(DeltaTime);

    FVector Steering = FVector::ZeroVector;

    switch (Behavior)
    {
    case ESteeringBehavior::Seek:
        if (TargetActor)
            Steering = Seek(TargetActor->GetActorLocation());
        break;

    case ESteeringBehavior::Flee:
        if (TargetActor)
            Steering = Flee(TargetActor->GetActorLocation());
        break;

    case ESteeringBehavior::Pursue:
        if (TargetActor)
            Steering = Pursue(TargetActor);
        break;

    case ESteeringBehavior::Evade:
        if (TargetActor)
            Steering = Evade(TargetActor);
        break;

    case ESteeringBehavior::Arrive:
        if (TargetActor)
            Steering = Arrive(TargetActor->GetActorLocation());
        break;

    case ESteeringBehavior::Wander:
        Steering = Wander(DeltaTime);
        break;

    case ESteeringBehavior::PathFollowing:
        Steering = PathFollowing();
        break;

    case ESteeringBehavior::Separation:
        Steering = Separation();
        break;

    case ESteeringBehavior::CollisionAvoidance:
        Steering = CollisionAvoidance();
        break;

    case ESteeringBehavior::ObstacleAvoidance:
        Steering = ObstacleAvoidance();
        break;

    case ESteeringBehavior::WallAvoidance:
        Steering = WallAvoidance();
        break;

    case ESteeringBehavior::Combined:
        Steering = Combined();
        break;
    }

    // Update velocity
    Velocity += Steering.GetClampedToMaxSize(MaxForce) * DeltaTime;
    Velocity = Velocity.GetClampedToMaxSize(MaxSpeed);
    Velocity.Z = 0.0f;

    // Calculate the new position
    FVector NewPosition = GetActorLocation() + Velocity * DeltaTime;

    // HARD BOUNDARY
    // Keep the entire agent inside the platform
    const float MaxX = FMath::Max(0.0f, WallLimitX - AgentRadius);
    const float MaxY = FMath::Max(0.0f, WallLimitY - AgentRadius);

    // Clamp position to the platform boundaries
    NewPosition.X = FMath::Clamp(NewPosition.X, -MaxX, MaxX);
    NewPosition.Y = FMath::Clamp(NewPosition.Y, -MaxY, MaxY);

    // Stop outward movement at the boundaries
    if (NewPosition.X >= MaxX && Velocity.X > 0.0f)
    {
        Velocity.X = 0.0f;
    }
    else if (NewPosition.X <= -MaxX && Velocity.X < 0.0f)
    {
        Velocity.X = 0.0f;
    }

    if (NewPosition.Y >= MaxY && Velocity.Y > 0.0f)
    {
        Velocity.Y = 0.0f;
    }
    else if (NewPosition.Y <= -MaxY && Velocity.Y < 0.0f)
    {
        Velocity.Y = 0.0f;
    }

    // Move the agent
    SetActorLocation(NewPosition);

    // Draw debug information
    if (bDrawDebug)
    {
        DrawDebugInfo();
    }
}

FVector ASteeringAgent::GetFlatLocation() const
{
    FVector Position = GetActorLocation();
    Position.Z = 0.0f;
    return Position;
}

FVector ASteeringAgent::GetFlatDirection() const
{
    FVector Direction = Velocity.GetSafeNormal2D();
    if (Direction.IsNearlyZero())
    {
        Direction = GetActorForwardVector().GetSafeNormal2D();
    }
    return Direction;
}

FVector ASteeringAgent::CalculateSteering(FVector DesiredVelocity)
{
    FVector Steering = DesiredVelocity - Velocity;
    Steering.Z = 0.0f;
    return Steering.GetClampedToMaxSize(MaxForce);
}

FVector ASteeringAgent::Seek(FVector Target)
{
    FVector Desired = Target - GetActorLocation();
    Desired.Z = 0.0f;
    return CalculateSteering(Desired.GetSafeNormal() * MaxSpeed);
}

FVector ASteeringAgent::Flee(FVector Target)
{
    FVector Desired = GetActorLocation() - Target;
    Desired.Z = 0.0f;
    return CalculateSteering(Desired.GetSafeNormal() * MaxSpeed);
}

FVector ASteeringAgent::Pursue(AActor* Target)
{
    if (!Target) return FVector::ZeroVector;

    FVector TargetVelocity = Target->GetVelocity();
    if (ASteeringAgent* OtherAgent = Cast<ASteeringAgent>(Target))
    {
        TargetVelocity = OtherAgent->Velocity;
    }
    TargetVelocity.Z = 0.0f;

    FVector ToTarget = Target->GetActorLocation() - GetActorLocation();
    ToTarget.Z = 0.0f;
    const float PredictionTime = FMath::Min(ToTarget.Size() / FMath::Max(MaxSpeed, 1.0f), 2.0f);
    return Seek(Target->GetActorLocation() + TargetVelocity * PredictionTime);
}

FVector ASteeringAgent::Evade(AActor* Target)
{
    if (!Target) return FVector::ZeroVector;

    FVector TargetVelocity = Target->GetVelocity();
    if (ASteeringAgent* OtherAgent = Cast<ASteeringAgent>(Target))
    {
        TargetVelocity = OtherAgent->Velocity;
    }
    TargetVelocity.Z = 0.0f;

    FVector ToTarget = Target->GetActorLocation() - GetActorLocation();
    ToTarget.Z = 0.0f;
    const float PredictionTime = FMath::Min(ToTarget.Size() / FMath::Max(MaxSpeed, 1.0f), 2.0f);
    return Flee(Target->GetActorLocation() + TargetVelocity * PredictionTime);
}

FVector ASteeringAgent::Arrive(FVector Target)
{
    FVector ToTarget = Target - GetActorLocation();
    ToTarget.Z = 0.0f;
    const float Distance = ToTarget.Size();

    if (Distance < 5.0f)
    {
        return CalculateSteering(FVector::ZeroVector);
    }

    float Speed = MaxSpeed;
    if (Distance < SlowRadius)
    {
        Speed *= Distance / FMath::Max(SlowRadius, 1.0f);
    }
    return CalculateSteering(ToTarget.GetSafeNormal() * Speed);
}

FVector ASteeringAgent::Wander(float DeltaTime)
{
    WanderAngle += FMath::FRandRange(-WanderJitter, WanderJitter) * DeltaTime;

    const FVector Forward = GetFlatDirection();
    const FVector CircleCenter = Forward * WanderDistance;
    const FVector Displacement(FMath::Cos(WanderAngle), FMath::Sin(WanderAngle), 0.0f);
    const FVector Desired = (CircleCenter + Displacement * WanderRadius).GetSafeNormal() * MaxSpeed;
    return CalculateSteering(Desired);
}

FVector ASteeringAgent::PathFollowing()
{
    if (PathPoints.IsEmpty()) return FVector::ZeroVector;

    CurrentPathIndex = CurrentPathIndex % PathPoints.Num();
    AActor* Point = PathPoints[CurrentPathIndex];
    if (!IsValid(Point)) return FVector::ZeroVector;

    FVector ToPoint = Point->GetActorLocation() - GetActorLocation();
    ToPoint.Z = 0.0f;

    if (ToPoint.Size() <= WaypointRadius)
    {
        CurrentPathIndex = (CurrentPathIndex + 1) % PathPoints.Num();
        Point = PathPoints[CurrentPathIndex];
        if (!IsValid(Point)) return FVector::ZeroVector;
    }

    return Seek(Point->GetActorLocation());
}

FVector ASteeringAgent::Separation()
{
    FVector Away = FVector::ZeroVector;
    int32 NearbyCount = 0;

    for (ASteeringAgent* Other : OtherAgents)
    {
        if (!IsValid(Other) || Other == this) continue;

        FVector Difference = GetActorLocation() - Other->GetActorLocation();
        Difference.Z = 0.0f;
        const float Distance = Difference.Size();

        if (Distance > 1.0f && Distance < SeparationRadius)
        {
            Away += Difference.GetSafeNormal() / Distance;
            NearbyCount++;
        }
    }

    if (NearbyCount == 0) return FVector::ZeroVector;
    Away /= NearbyCount;
    return CalculateSteering(Away.GetSafeNormal() * MaxSpeed);
}

FVector ASteeringAgent::CollisionAvoidance()
{
    const FVector MyPosition = GetFlatLocation();
    const FVector MyFuture = MyPosition + Velocity * CollisionLookAhead;

    for (ASteeringAgent* Other : OtherAgents)
    {
        if (!IsValid(Other) || Other == this) continue;

        FVector OtherPosition = Other->GetFlatLocation();
        const FVector OtherFuture = OtherPosition + Other->Velocity * CollisionLookAhead;
        const FVector Difference = MyFuture - OtherFuture;

        if (Difference.Size2D() < AgentRadius * 2.0f)
        {
            // Choose a sideways direction to pass the other agent.
            FVector RelativeVelocity = Velocity - Other->Velocity;
            FVector Side(-RelativeVelocity.Y, RelativeVelocity.X, 0.0f);
            if (Side.IsNearlyZero())
            {
                Side = FVector(-GetFlatDirection().Y, GetFlatDirection().X, 0.0f);
            }
            if (Side.IsNearlyZero()) Side = FVector::YAxisVector;
            return CalculateSteering(Side.GetSafeNormal() * MaxSpeed);
        }
    }
    return FVector::ZeroVector;
}

FVector ASteeringAgent::ObstacleAvoidance()
{
    const FVector Direction = GetFlatDirection();
    const FVector Position = GetFlatLocation();

    for (AActor* Obstacle : Obstacles)
    {
        if (!IsValid(Obstacle) || Obstacle == this) continue;

        FVector ToObstacle = Obstacle->GetActorLocation() - Position;
        ToObstacle.Z = 0.0f;
        const float ForwardDistance = FVector::DotProduct(ToObstacle, Direction);
        if (ForwardDistance < 0.0f || ForwardDistance > ObstacleLookAhead) continue;

        const FVector ClosestPoint = Position + Direction * ForwardDistance;
        FVector Offset = Obstacle->GetActorLocation() - ClosestPoint;
        Offset.Z = 0.0f;

        // Approximate the obstacle as a circle in the XY plane.
        const FVector Bounds = Obstacle->GetComponentsBoundingBox().GetExtent();
        const float Radius = FMath::Max(Bounds.X, Bounds.Y) + ObstacleExtraRadius;

        if (Offset.Size() < Radius)
        {
            FVector Side(-Direction.Y, Direction.X, 0.0f);
            if (FVector::DotProduct(Side, ToObstacle) > 0.0f) Side *= -1.0f;
            return CalculateSteering(Side * MaxSpeed);
        }
    }
    return FVector::ZeroVector;
}

FVector ASteeringAgent::WallAvoidance()
{
    const FVector Position = GetFlatLocation();
    FVector Desired = FVector::ZeroVector;

    const float SafeX = FMath::Max(0.0f, WallLimitX - WallMargin);
    const float SafeY = FMath::Max(0.0f, WallLimitY - WallMargin);

    if (Position.X > SafeX) Desired.X = -MaxSpeed;
    else if (Position.X < -SafeX) Desired.X = MaxSpeed;

    if (Position.Y > SafeY) Desired.Y = -MaxSpeed;
    else if (Position.Y < -SafeY) Desired.Y = MaxSpeed;

    if (Desired.IsNearlyZero()) return FVector::ZeroVector;
    return CalculateSteering(Desired.GetSafeNormal() * MaxSpeed);
}

FVector ASteeringAgent::Combined()
{
    // Task 3: Combining Steering Behaviors
    // First priority: avoid walls
    FVector WallForce = WallAvoidance();

    if (!WallForce.IsNearlyZero())
    {
        return WallForce;
    }

    // Second priority: avoid obstacles
    FVector ObstacleForce = ObstacleAvoidance();

    if (!ObstacleForce.IsNearlyZero())
    {
        return ObstacleForce;
    }

    // Third priority: avoid collisions with other agents
    FVector CollisionForce = CollisionAvoidance();

    if (!CollisionForce.IsNearlyZero())
    {
        return CollisionForce;
    }

    // Follow the path while keeping distance from other agents
    FVector Steering = PathFollowing();
    Steering += Separation() * 1.5f;

    return Steering.GetClampedToMaxSize(MaxForce);
}


void ASteeringAgent::DrawDebugInfo()
{
    const FVector Position = GetActorLocation();
    const FVector Direction = GetFlatDirection();
    DrawDebugDirectionalArrow(GetWorld(), Position + FVector(0, 0, 40),
        Position + FVector(0, 0, 40) + Direction * 120.0f,
        30.0f, FColor::Blue, false, 0.0f, 0, 3.0f);

    if (Behavior == ESteeringBehavior::PathFollowing || Behavior == ESteeringBehavior::Combined)
    {
        for (int32 Index = 0; Index < PathPoints.Num(); Index++)
        {
            if (!IsValid(PathPoints[Index])) continue;
            const FVector Point = PathPoints[Index]->GetActorLocation();
            DrawDebugSphere(GetWorld(), Point, WaypointRadius, 12,
                Index == CurrentPathIndex ? FColor::Yellow : FColor::Green,
                false, 0.0f, 0, 2.0f);
            if (PathPoints.Num() > 1)
            {
                AActor* Next = PathPoints[(Index + 1) % PathPoints.Num()];
                if (IsValid(Next))
                {
                    DrawDebugLine(GetWorld(), Point, Next->GetActorLocation(),
                        FColor::Green, false, 0.0f, 0, 2.0f);
                }
            }
        }
    }

    if (Behavior == ESteeringBehavior::Separation || Behavior == ESteeringBehavior::Combined)
    {
        DrawDebugCircle(GetWorld(), Position, SeparationRadius, 32,
            FColor::Cyan, false, 0.0f, 0, 1.0f,
            FVector(1, 0, 0), FVector(0, 1, 0), false);
    }

    if (Behavior == ESteeringBehavior::ObstacleAvoidance || Behavior == ESteeringBehavior::Combined)
    {
        DrawDebugLine(GetWorld(), Position, Position + Direction * ObstacleLookAhead,
            FColor::Red, false, 0.0f, 0, 2.0f);
    }

    if (Behavior == ESteeringBehavior::WallAvoidance || Behavior == ESteeringBehavior::Combined)
    {
        const float SafeX = FMath::Max(0.0f, WallLimitX - WallMargin);
        const float SafeY = FMath::Max(0.0f, WallLimitY - WallMargin);
        const float Z = Position.Z;
        const FVector A(-SafeX, -SafeY, Z), B(SafeX, -SafeY, Z);
        const FVector C(SafeX, SafeY, Z), D(-SafeX, SafeY, Z);
        DrawDebugLine(GetWorld(), A, B, FColor::Orange, false, 0.0f, 0, 2.0f);
        DrawDebugLine(GetWorld(), B, C, FColor::Orange, false, 0.0f, 0, 2.0f);
        DrawDebugLine(GetWorld(), C, D, FColor::Orange, false, 0.0f, 0, 2.0f);
        DrawDebugLine(GetWorld(), D, A, FColor::Orange, false, 0.0f, 0, 2.0f);
    }
}
